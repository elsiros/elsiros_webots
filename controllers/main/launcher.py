from Soccer.strategy import *
from gcreceiver import ThreadedGameStateReceiver


def init_gcreceiver(team, player, is_goalkeeper):
    receiver = ThreadedGameStateReceiver(team, player, is_goalkeeper)    
    receiver.start() # Strat receiving and answering
    return receiver

def player_super_cycle():
    pass




