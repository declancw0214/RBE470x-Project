import matplotlib.pyplot as plt
from IPython import display

plt.ion()

def plot(n_games,scores, mean_scores,loses, wins):
    print("plotting")
    display.clear_output(wait=True)
    display.display(plt.gcf())
    plt.clf()
    plt.subplot(211)
    plt.title('Training...')
    plt.xlabel('Number of Games')
    plt.ylabel('Score')
    games = [0,n_games]
    plt.plot(games, scores, label='scores')
    plt.plot(games, mean_scores, label='mean scores')

    plt.ylim(ymin=0)
    plt.text(len(scores)-1, scores[-1], str(scores[-1]))
    plt.text(len(mean_scores)-1, mean_scores[-1], str(mean_scores[-1]))
    plt.subplot(212)
    plt.title('Record')
    labels=["wins","loses"]
    data= [wins,loses]
    plt.pie(data,labels=labels)
    plt.show(block=False)
    plt.pause(.1)