import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("../logs/train_history.csv")
plt.figure(figsize=(10, 5))
plt.plot(data['Episode'], data['Reward'])
plt.title('Quá trình học của Robot (Reward Over Time)')
plt.xlabel('Episode')
plt.ylabel('Total Reward')
plt.grid(True)
plt.savefig("../logs/learning_curve.png")
plt.show()