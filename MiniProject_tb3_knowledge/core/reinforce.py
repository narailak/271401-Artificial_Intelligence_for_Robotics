"""Simulation-only REINFORCE for the existing four bounded policy weights."""
from dataclasses import replace
import math
import random

from .navigation import Navigator,wrap
from .simulation import run_episode

WEIGHT_KEYS=('goal_weight','clearance_weight','turn_weight','revisit_weight')


def action_distribution(candidates,features,previous_heading,hysteresis,temperature):
    """Return probabilities and exact d(log pi)/d(weight) for masked headings."""
    if not math.isfinite(temperature) or temperature<=0:raise ValueError('invalid exploration temperature')
    costs=[candidate[0] for candidate in candidates]
    previous=min(range(len(candidates)),key=lambda i:(abs(wrap(candidates[i][1]-previous_heading)),candidates[i]))
    costs[previous]-=hysteresis
    best=min(costs)
    probabilities=[math.exp(-(cost-best)/temperature) for cost in costs]
    total=sum(probabilities)
    probabilities=[p/total for p in probabilities]
    expected=[sum(p*f[j] for p,f in zip(probabilities,features)) for j in range(4)]
    gradients=[tuple((expected[j]-f[j])/temperature for j in range(4)) for f in features]
    return probabilities,gradients


class ExploringNavigator(Navigator):
    def __init__(self,arena,goal,policy,hardware,*,seed,temperature):
        super().__init__(arena,goal,policy,hardware)
        self.rng=random.Random(seed)
        self.temperature=temperature
        self.gradient=None
        self.transitions=[]

    def command(self,observation,now):
        self.gradient=None
        return super().command(observation,now)

    def _select_candidate(self,candidates,features):
        probabilities,gradients=action_distribution(candidates,features,self._last_heading,
            self.policy.hysteresis,self.temperature)
        index=self.rng.choices(range(len(candidates)),weights=probabilities,k=1)[0]
        self.gradient=gradients[index]
        return candidates[index]

    def observe(self,previous,current,terminal,dt):
        before=math.dist((previous.x,previous.y),self.goal)
        after=math.dist((current.x,current.y),self.goal)
        reward=10*(before-after)-.02*dt
        reward+=dict(success=20.,collision=-30.,estimated_arrival_error=-10.,
            recovery_exhausted=-8.,outside_safe_arena=-15.,timeout=-8.).get(terminal,0.)
        self.transitions.append((self.gradient,reward))


def reinforce_update(policy,transitions,baseline,learning_rate=.08):
    """Use discounted reward-to-go and a baseline from earlier episodes only."""
    returns=[];value=0.
    for _,reward in reversed(transitions):
        value=reward+.995*value
        returns.append(value)
    returns.reverse()
    if not returns:return policy,baseline,0.
    gradient=[0.]*4
    count=sum(g is not None for g,_ in transitions)
    for (log_gradient,_),value in zip(transitions,returns):
        if log_gradient is None:continue
        for j in range(4):gradient[j]+=(value-baseline)*log_gradient[j]/max(1,count)
    norm=math.sqrt(sum(v*v for v in gradient))
    scale=learning_rate/max(1.,norm)
    weights={key:max(0.,min(10.,getattr(policy,key)+scale*gradient[j])) for j,key in enumerate(WEIGHT_KEYS)}
    updated=replace(policy,**weights)
    baseline=.9*baseline+.1*sum(returns)/len(returns)
    return updated,baseline,norm


def learn_weights(world,arena,start,goal,hardware,base,progress,episodes=12):
    policy=base;baseline=0.;checkpoints=[];history=[]
    for index in range(episodes):
        seed=3001+index
        navigator=ExploringNavigator(arena,goal,policy,hardware,seed=6001+index,
            temperature=.12-.06*index/max(1,episodes-1))
        before={key:getattr(policy,key) for key in WEIGHT_KEYS}
        result=run_episode(world,arena,start,goal,policy,hardware,seed=seed,
            navigator=navigator,on_step=navigator.observe)
        policy,baseline,norm=reinforce_update(policy,navigator.transitions,baseline)
        history.append(dict(episode=index+1,seed=seed,status=result['status'],
            reward=sum(r for _,r in navigator.transitions),gradient_norm=norm,
            updates=sum(g is not None for g,_ in navigator.transitions),
            before=before,after={key:getattr(policy,key) for key in WEIGHT_KEYS}))
        progress(f'เรียนรู้ weight RL {index+1}/{episodes} · {result["status"]}')
        if (index+1)%4==0 or index+1==episodes:checkpoints.append(policy)
    return checkpoints,dict(method='REINFORCE',weight_keys=list(WEIGHT_KEYS),episodes=history)
