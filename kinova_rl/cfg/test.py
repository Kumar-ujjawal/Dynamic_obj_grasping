import pandas as pd

# Assuming 'j.csv' is in the same directory as your script
df = pd.read_csv(r"/home/vr-lab/kinova_arm/src/kinova_rl/cfg/j.csv")
print(df.head(5))