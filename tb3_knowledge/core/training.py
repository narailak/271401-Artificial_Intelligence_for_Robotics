"""Small reproducible policy search; evaluation never influences selection."""
from dataclasses import asdict, replace
import time

from .model import Hardware, Policy
from .simulation import run_episode


def policy_candidates():
    base = Policy()
    return [base,
        replace(base, preferred_side=-1, clearance_weight=1.2, goal_weight=2.4, lookahead=0.55),
        replace(base, preferred_side=-1, clearance_weight=0.45, revisit_weight=0.7, wall_distance=0.22),
        replace(base, clearance_weight=1.5, goal_weight=2.4, revisit_weight=1.7, turn_weight=0.4, progress_window=12),
        replace(base, preferred_side=-1, goal_weight=2.0, revisit_weight=2.0, lookahead=0.45, progress_window=10),
        replace(base, goal_weight=3.5, clearance_weight=0.5, lookahead=0.8),
        replace(base, preferred_side=-1, turn_weight=0.7, side_hold=4.0, hysteresis=0.08, lookahead=0.7),
        replace(base, goal_weight=2.6, clearance_weight=1.0, revisit_weight=2.5, side_hold=3.0, progress_window=8)]


def _compact(episode):
    return {key: episode[key] for key in ('status', 'time', 'distance', 'collisions', 'goal_error', 'recoveries', 'seed', 'noise',
        'estimated_goal_error', 'physical_goal_tolerance') if key in episode}


def _score(episodes):
    success = sum(e['status'] == 'success' for e in episodes)
    collision = sum(e['collisions'] for e in episodes)
    failure_error = sum(e['goal_error'] for e in episodes if e['status'] != 'success')
    return (collision, -success, failure_error,
        sum(e['time'] for e in episodes), sum(e['distance'] for e in episodes), sum(e['recoveries'] for e in episodes))


def train(world, arena, start, goal, hardware: Hardware,
          candidate_limit=4, progress=None):
    """Return (selected Policy, report), with train and held-out noise seeds.

    The disclosed Start/Goal stays fixed. Complete every selected candidate,
    and always perform independent evaluation afterwards.
    Progress receives a human-readable string. Reports may retain trial scores;
    Knowledge exports must keep only the schema's aggregate scalar allowlist.
    """
    if not isinstance(candidate_limit, int) or not 1 <= candidate_limit <= len(policy_candidates()):
        raise ValueError("Candidate limit must be 1..8")
    if not world.reachable(start, goal, hardware.safe_radius):
        raise ValueError("Start and Goal are not connected with robot clearance")
    notify = progress or (lambda message: None)
    began = time.monotonic()
    trials = []
    training_seeds = (1701, 1702)
    evaluation_seeds = (9101, 9102)
    candidates = policy_candidates()[:candidate_limit]
    best_score, selected, selected_index, selected_episodes = None, candidates[0], 0, []
    for index, candidate in enumerate(candidates):
        notify(f"ทดลองพารามิเตอร์ {index + 1}/{len(candidates)} · 2 รอบต่อชุด")
        episodes = [run_episode(world, arena, start, goal, candidate, hardware, seed=seed) for seed in training_seeds]
        score = _score(episodes)
        trials.append({'index': index, 'policy': asdict(candidate), 'episodes': [_compact(e) for e in episodes]})
        if best_score is None or score < best_score:
            best_score, selected, selected_index, selected_episodes = score, candidate, index, episodes
        notify(f"ชุด {index + 1}: ถึงเป้าหมาย {sum(e['status'] == 'success' for e in episodes)}/2 · ชน {sum(e['collisions'] for e in episodes)}")
    training_seconds = time.monotonic() - began
    notify("ประเมินชุดที่เลือกและค่าเริ่มต้นด้วย seed ใหม่; ไม่ใช้ผลนี้เลือกพารามิเตอร์")
    evaluation = [run_episode(world, arena, start, goal, selected, hardware, seed=seed) for seed in evaluation_seeds]
    if selected == Policy():
        baseline = evaluation
    else:
        baseline = [run_episode(world, arena, start, goal, Policy(), hardware, seed=seed) for seed in evaluation_seeds]
    report = {
        'cases': len(selected_episodes), 'successes': sum(e['status'] == 'success' for e in selected_episodes),
        'collisions': sum(e['collisions'] for e in selected_episodes),
        'evaluated_candidates': len(trials), 'training_seconds': round(training_seconds, 3),
        'selected_index': selected_index, 'train_seed': training_seeds[0], 'evaluation_seed': evaluation_seeds[0],
        'evaluation_cases': len(evaluation), 'evaluation_successes': sum(e['status'] == 'success' for e in evaluation),
        'evaluation_collisions': sum(e['collisions'] for e in evaluation),
        'baseline_cases': len(baseline), 'baseline_successes': sum(e['status'] == 'success' for e in baseline),
        'baseline_collisions': sum(e['collisions'] for e in baseline),
        'mean_time': round(sum(e['time'] for e in evaluation) / len(evaluation), 4),
        'mean_distance': round(sum(e['distance'] for e in evaluation) / len(evaluation), 5),
        'trials': trials, 'evaluation': [_compact(e) for e in evaluation],
        'baseline': [_compact(e) for e in baseline], 'total_seconds': round(time.monotonic() - began, 3),
        'evaluation_seeds': list(evaluation_seeds), 'training_seeds': list(training_seeds),
    }
    notify(f"เสร็จ: ชุด {selected_index + 1} · ทดสอบอิสระ {report['evaluation_successes']}/{len(evaluation)}")
    return selected, report
