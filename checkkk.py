import joblib
import neccessary_method as nm

regY = joblib.load('randomForest_Model_for_dataY.sav')
regZ = joblib.load('GradientBoostingRegressor_Model_for_dataZ.sav')

X, Y, Z = nm.getXYZmm(284, 194, 240, regY, regZ)
print(f"X = {round(X, 2)} mm, Y = {round((Y*10), 2)} mm, Z = {round((Z), 2)} mm")