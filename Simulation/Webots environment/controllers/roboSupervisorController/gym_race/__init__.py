from gym.envs.registration import register

register(
    id='race-v0',
    entry_point='gym_race.envs:RaceEnv',
)
