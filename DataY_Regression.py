import pandas as pd
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
import joblib

df=pd.read_csv(r"dataY.csv")

print(df.info())
print(df.dtypes)

X = df.drop(df.loc[:, ["Milimeters"]].columns, axis=1)
y = df['Milimeters']

x_train, x_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=1)
reg = RandomForestRegressor(n_estimators=100, random_state=1)
reg.fit(X, y)

# make predictions using the testing set
y_pred = reg.predict(x_test)

# calculate mean squared error
mse = mean_squared_error(y_test, y_pred)
print("Mean Square Error: ", mse)

print(f"Pred for Y = 165: {reg.predict([[165]])} mm")

# filename = 'randomForest_Model_for_dataY.sav'
# joblib.dump(reg, filename)