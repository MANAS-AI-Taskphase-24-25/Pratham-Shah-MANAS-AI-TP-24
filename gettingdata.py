import pandas as pd
import numpy as np

do = pd.read_csv("Train.csv")
test_do = pd.read_csv("Test.csv")
tdata = []
data = []
needed_titles = list(do)[1:-1]
#exclues sr no, name, and price
prices = do['Price']
tprices = test_do['Price']

test_df = test_do.copy()
df = do.copy()

for index, name in enumerate(df["Name"]):
    df.loc[index, "Name"] = str(df.loc[index, "Name"]).lower().split(" ")[0]
for index, name in enumerate(test_df["Name"]):
    test_df.loc[index, "Name"] = str(test_df.loc[index, "Name"]).lower().split(" ")[0]

test_df = pd.get_dummies(test_df[needed_titles], columns = ["Name", "Location", "Fuel_Type" , "Transmission", "Owner_Type", "Year"], dtype=int)
df = pd.get_dummies(df[needed_titles], columns = ["Name", "Location", "Fuel_Type" , "Transmission", "Owner_Type", "Year"], dtype=int)

for index, vals in df.iterrows():
    df = df.replace(df["Engine"][index], float(str(df["Engine"][index]).replace(" CC", ""))).replace(df["Mileage"][index], float(str(df["Mileage"][index]).replace(" kmpl", "").replace("nan", "0"))).replace(df["Power"][index], float(str(df["Power"][index]).replace(" bhp", "").replace("null", "-1")))
    df.infer_objects(copy=False)

for tindex, tvals in test_df.iterrows():
    test_df = test_df.replace(test_df["Engine"][tindex], float(str(test_df["Engine"][tindex]).replace(" CC", ""))).replace(test_df["Mileage"][tindex], float(str(test_df["Mileage"][tindex]).replace(" kmpl", "").replace("nan", "0"))).replace(test_df["Power"][tindex], float(str(test_df["Power"][tindex]).replace(" bhp", "").replace("null", "-1")))
    test_df.infer_objects(copy=False)

test_df_titles = list(test_df)
df_titles = list(df)

for index, value in enumerate(df_titles):
    if test_df_titles.count(value) == 0:
        test_df.insert(index, value, list(0 for val in range(0, len(test_df))), True)

for index, value in df.iterrows():
    if(df["Power"][index] == -1):
        df.drop(index, inplace=True)
        prices.drop(index=index, inplace=True)
for index, value in test_df.iterrows():
    if(test_df["Power"][index] == -1):
        test_df.drop(index, inplace=True)
        tprices.drop(index=index, inplace=True)

data = df.values.tolist()
tdata = test_df.values.tolist()

if len(data[0]) == len(tdata[0]) and len(prices) == len(data):
    print(f" PARSING SUCESSFUL ({len(data[0])} features) " * 5)
    if(len(do)-len(df) != 0):
        print(f"CAUTION: {len(do) - len(df)} items removed from training set, {len(test_do)-len(test_df)} items removed from testing set.")
else:
    print(" WARNING: PARSING FAILURE " * 5)

print(len(df))