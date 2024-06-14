import numpy as np
import json

q = np.array([[0.0, 1.0, 3.0], [1.2, 34.5, 5.7]])

x = ['q', 'x_center', 'y_center']
y = [q.tolist(), 1.0, 2.0]

myDict = {}
for i in range(len(x)):
	myDict[x[i]] = y[i]

jsonText = json.dumps(myDict)

decodedText = json.loads(jsonText)

print(decodedText)

print(np.asarray(decodedText["q"]))