"""Policy-gradient math, frozen base, independent evaluation and legacy schema."""
from dataclasses import asdict,replace
import math
import unittest
from unittest.mock import patch

from tb3_knowledge.core.model import Arena,Hardware,Knowledge,Policy,Pose,SUMMARY_KEYS
from tb3_knowledge.core.navigation import Navigator,Observation
from tb3_knowledge.core.reinforce import (WEIGHT_KEYS,ExploringNavigator,action_distribution,
    reinforce_update,learn_weights)
from tb3_knowledge.core.simulation import run_episode
from tb3_knowledge.core.training import train
from tb3_knowledge.tests.test_navigation import AnalyticWorld


class ReinforceTests(unittest.TestCase):
    def test_exploration_keeps_hard_clearance_checks(self):
        nav=ExploringNavigator(Arena(2,2),(1.5,1.5),Policy(),Hardware(),seed=1,temperature=.1)
        obs=Observation(Pose(.4,.4),(.03,)*120)
        self.assertEqual(nav.command(obs,0.),(0.,0.))
        self.assertEqual(nav.reason,'footprint_clearance')
        self.assertIsNone(nav.gradient)

    def test_log_policy_gradient_matches_finite_differences(self):
        features=[(.1,.8,.2,0.),(.4,.1,.3,.2),(.8,.3,.1,.7)]
        weights=[3.,.8,.25,1.]
        def distribution(w):
            candidates=[(sum(a*b for a,b in zip(w,f))+.2*i,i*.2,1.) for i,f in enumerate(features)]
            return action_distribution(candidates,features,.2,.025,.3)
        probabilities,gradients=distribution(weights)
        self.assertAlmostEqual(sum(probabilities),1.)
        for j in range(4):
            plus=weights[:];minus=weights[:];plus[j]+=1e-5;minus[j]-=1e-5
            for i in range(3):
                derivative=(math.log(distribution(plus)[0][i])-math.log(distribution(minus)[0][i]))/2e-5
                self.assertAlmostEqual(derivative,gradients[i][j],places=6)
            self.assertAlmostEqual(sum(p*g[j] for p,g in zip(probabilities,gradients)),0.)

    def test_positive_reward_updates_weights_and_freezes_other_parameters(self):
        base=replace(Policy(),preferred_side=-1,lookahead=.45,progress_window=10)
        updated,baseline,norm=reinforce_update(base,[((-1.,1.,0.,0.),5.)],0.)
        self.assertLess(updated.goal_weight,base.goal_weight)
        self.assertGreater(updated.clearance_weight,base.clearance_weight)
        self.assertGreater(norm,0.)
        self.assertGreater(baseline,0.)
        for key,value in asdict(base).items():
            if key not in WEIGHT_KEYS:self.assertEqual(getattr(updated,key),value)
        bounded,_,_=reinforce_update(replace(base,goal_weight=0.,clearance_weight=10.),[((-100.,100.,0.,0.),5.)],0.)
        self.assertEqual(bounded.goal_weight,0.)
        self.assertEqual(bounded.clearance_weight,10.)

    def test_real_short_rollouts_learn_new_reproducible_weights(self):
        arena=Arena(2,2);hardware=Hardware();base=Policy()
        def short(*args,**kwargs):return run_episode(*args,**kwargs,max_time=2.)
        with patch('tb3_knowledge.core.reinforce.run_episode',side_effect=short):
            checkpoints,report=learn_weights(AnalyticWorld(),arena,Pose(.4,.4),(1.5,1.5),hardware,base,lambda s:None,episodes=4)
            repeated,again=learn_weights(AnalyticWorld(),arena,Pose(.4,.4),(1.5,1.5),hardware,base,lambda s:None,episodes=4)
        self.assertEqual(checkpoints,repeated)
        self.assertEqual(report,again)
        self.assertNotEqual(checkpoints[-1],base)
        self.assertTrue(all(e['updates']>0 and math.isfinite(e['gradient_norm']) for e in report['episodes']))
        for policy in checkpoints:
            for key,value in asdict(base).items():
                if key not in WEIGHT_KEYS:self.assertEqual(getattr(policy,key),value)

    def test_default_selection_keeps_greedy_hysteresis_and_tie_order(self):
        nav=Navigator(Arena(2,2),(1.5,1.5),Policy(),Hardware())
        candidates=[(.11,.5,.6),(.10,0.,.5),(.09,-.5,.4)]
        selected=nav._select_candidate(candidates,[])
        self.assertEqual(selected,(.10,0.,.5))
        self.assertEqual(candidates,sorted(candidates))

    def test_all_twelve_learning_episodes_keep_selected_start_and_goal(self):
        start=Pose(.4,.4,.7);goal=(1.5,1.5)
        with patch('tb3_knowledge.core.reinforce.run_episode',return_value={'status':'timeout'}) as episode:
            checkpoints,report=learn_weights(AnalyticWorld(),Arena(2,2),start,goal,
                Hardware(),Policy(),lambda s:None)
        self.assertEqual(episode.call_count,12)
        self.assertEqual(len(checkpoints),3)
        self.assertEqual(len(report['episodes']),12)
        for index,call in enumerate(episode.call_args_list):
            self.assertEqual(call.args[2:4],(start,goal))
            self.assertEqual(call.kwargs['seed'],3001+index)
            self.assertEqual(call.kwargs['navigator'].goal,goal)

    def test_training_selects_base_with_common_weights_then_uses_greedy_checkpoints(self):
        calls=[];frozen=[]
        def episode(world,arena,start,goal,policy,hardware,seed=0):
            calls.append((seed,asdict(policy),start,goal))
            # Checkpoint wins selection but loses held-out evaluation.
            success=(policy.goal_weight==4.) if seed<9000 else (policy.goal_weight!=4.)
            return dict(status='success' if success else 'timeout',time=10.,distance=1.,collisions=0,
                goal_error=.05 if success else .5,recoveries=0,seed=seed,noise=True)
        def learned(world,arena,start,goal,hardware,base,progress):
            frozen.append(base)
            return [replace(base,goal_weight=4.)],dict(method='REINFORCE')
        with patch('tb3_knowledge.core.training.run_episode',side_effect=episode), \
                patch('tb3_knowledge.core.training.learn_weights',side_effect=learned):
            selected,report=train(AnalyticWorld(),Arena(2,2),Pose(.4,.4),(1.5,1.5),Hardware(),candidate_limit=8)
        common={key:getattr(Policy(),key) for key in WEIGHT_KEYS}
        self.assertTrue(all({key:p[key] for key in WEIGHT_KEYS}==common for _,p,_,_ in calls[:16]))
        self.assertTrue(all((initial,target)==(Pose(.4,.4),(1.5,1.5))
            for _,_,initial,target in calls))
        self.assertEqual([seed for seed,_,_,_ in calls[:16]],[1701,1702]*8)
        self.assertEqual(selected.goal_weight,4.)
        self.assertEqual(report['evaluation_successes'],0)
        self.assertEqual(report['rl']['selected_source'],'reinforce')
        first_eval=next(i for i,c in enumerate(calls) if c[0]>=9000)
        self.assertTrue(all(c[0]>=9000 for c in calls[first_eval:]))
        self.assertEqual(report['evaluated_candidates'],8)
        for key,value in asdict(frozen[0]).items():
            if key not in WEIGHT_KEYS:self.assertEqual(getattr(selected,key),value)
        legacy=Knowledge(Arena(2,2),Pose(.4,.4),(1.5,1.5)).document()
        summary={key:value for key,value in report.items() if key in SUMMARY_KEYS}
        document=Knowledge(Arena(2,2),Pose(.4,.4),(1.5,1.5),selected,training_summary=summary).document()
        self.assertEqual(document.keys(),legacy.keys())
        for key in ('policy','hardware','initial_reference','mission'):self.assertEqual(document[key].keys(),legacy[key].keys())
        self.assertNotIn('rl',document['training_summary'])
        self.assertEqual(Knowledge.from_document(document).policy,selected)

    def test_greedy_regression_retains_base_instead_of_exporting_worse_rl_weights(self):
        def episode(world,arena,start,goal,policy,hardware,seed=0):
            return dict(status='success' if policy.goal_weight==3. else 'collision',
                time=10.,distance=1.,collisions=int(policy.goal_weight!=3.),goal_error=.05,recoveries=0,seed=seed,noise=True)
        with patch('tb3_knowledge.core.training.run_episode',side_effect=episode), \
                patch('tb3_knowledge.core.training.learn_weights',return_value=([replace(Policy(),goal_weight=4.)],{})):
            selected,report=train(AnalyticWorld(),Arena(2,2),Pose(.4,.4),(1.5,1.5),Hardware(),candidate_limit=1)
        self.assertEqual(selected,Policy())
        self.assertEqual(report['rl']['selected_source'],'base_fallback')


if __name__=='__main__':unittest.main()
