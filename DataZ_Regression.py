import pandas as pd
# from sklearn.ensemble import RandomForestRegressor
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
import joblib

df=pd.read_csv(r"dataZ.csv")

print(df.info())
print(df.dtypes)

X = df.drop(df.loc[:, ["Z", "MM", "Higher Y", "Dista"]].columns, axis=1)
y = df['Z']


x_train, x_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=1)
# reg = RandomForestRegressor(n_estimators=200, random_state=0)
reg = GradientBoostingRegressor()
reg.fit(X, y)

# make predictions using the testing set
y_pred = reg.predict(x_test)

# calculate mean squared error
mse = mean_squared_error(y_test, y_pred)
print("Mean Square Error: ", mse)

# Checking
yy = 240
yy_higher = 61

print(f"Pred for Z: {reg.predict([[yy, yy-yy_higher]])} mm")

filename = 'GradientBoostingRegressor_Model_for_dataZ.sav'
joblib.dump(reg, filename)