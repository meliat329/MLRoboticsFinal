import csv 


#setup
total = 0
correct = 0
lowdif = 1000
def difference(x, y):
    return abs((x[0] - y[0])) + abs((x[1] - y[1])) + abs((x[2] - y[2]))


#open the test data sensor values list and scan through one at a time to get the machine learning response
test_data_object = csv.reader(open("test_data_A.csv", 'r'), delimiter = ",")
for input_row in test_data_object:
    for i in range(0,4):
        input_row[i] = float(input_row[i])

    traning_data_object = csv.reader(open("training_data_B.csv", 'r'), delimiter = ",")
    for training_row in traning_data_object:
        for j in range(0,4):
            training_row[j] = float(training_row[j])
        
        dif = difference(input_row, training_row)
        if dif < lowdif:
            lowdif = dif
            currturn = training_row[3]

    #check to see if that response is the correct response 
    if currturn == input_row[3]:
        correct += 1 
        total += 1 
    else:
        total += 1
    lowdif = 1000

#use these values to compute the percentage correct for all situations 
# provided by the test data for the given training data
print("correct: ", correct, "out of: ", total)



