import csv
import random

sensor_sets = {}
# for each set of sets { (Left, Center, Right): TURN } TURN is 1: left, 2: right, 3: around, 4: none
sensor_sets["training_data_A"] = {} 
sensor_sets["training_data_B"] = {}
sensor_sets["test_data_A"] = {}
sensor_sets["test_data_B"] = {}

def get_turn_from_sensors(sensor_set):
    wall_dist = 0.4
    wall_dist_margin_of_error = 0.01
    #Get New Sensor Distance Values
    left_dist = sensor_set[0] #self.getSensorValue("left")
    right_dist = sensor_set[2] #self.getSensorValue("right")
    front_dist = sensor_set[1] #self.getSensorValue("front")

    #Form Boolean Symbols from Sensor Data
    right_covered = (right_dist > 0 and right_dist < (wall_dist + wall_dist_margin_of_error))
    left_covered = (left_dist > 0 and left_dist < (wall_dist + wall_dist_margin_of_error))
    front_covered = (front_dist > 0 and front_dist < (wall_dist + wall_dist_margin_of_error))
    #print("Left Covered: %f; Right Covered: %f; Front Covered: %f;\n" %(left_covered, right_covered, front_covered))

    if left_covered and right_covered and front_covered:
        return 3 #turn around
        
    elif left_covered and front_covered: 
        return 2 #turn right

    elif right_covered and front_covered:
        return 1 #turn left

    elif front_covered: #not a dead end but a single obstacle exists
        return 1 #turn left TODO: is this the behavior we want? 

    else: #no sensors activated or tunnel 
        return 4 #none  

#training A
for i in range(10):
    #generate random sensor values for a robot placed in the center of the cell
    generated_over = 0.5 + 0.09*random.random()
    generated_under = 0.5 + 0.09*random.random()
    #right side wall
    sensor_set = ( -1000, -1000, generated_under )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( -1000, -1000, generated_over )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #left side wall
    sensor_set = ( generated_under, -1000, -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( generated_over, -1000, -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #front obstacle
    sensor_set = ( -1000, generated_under,  -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set)
    sensor_set = ( -1000, generated_under,  -1000 ) 
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set)  

    #left corner obstacle
    sensor_set = ( generated_under, generated_over,  -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( generated_over, generated_under,  -1000 ) 
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #right corner obstacle
    sensor_set = ( -1000, generated_over, generated_under )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( -1000, generated_under, generated_over )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #dead end obstacle
    sensor_set = ( generated_under, generated_over, generated_over )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( generated_over, generated_under, generated_under )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #no sensors 
    sensor_set = ( -1000, -1000, -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 


#training B
for i in range(1000):
    location = i%10 * 0.1 #decimeter range from sensors touching the wall to edge of the cell
    #generate random sensor values for a robot placement
    generated_num = location + 0.09*random.random()
    #right side wall 
    sensor_set = ( -1000, -1000, generated_num ) 
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set)

    #left side wall
    sensor_set = ( generated_num, -1000, -1000 )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #front obstacle
    sensor_set = ( -1000, generated_num,  -1000 ) 
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set)


    #left corner obstacle
    sensor_set = ( generated_num, generated_num,  -1000 ) 
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #right corner obstacle
    sensor_set = ( -1000, generated_num, generated_num )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #dead end obstacle
    sensor_set = ( generated_num, generated_num, generated_num )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #no sensors 
    sensor_set = ( -1000, -1000, -1000 )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

#test data A
for i in range(10):
    #COPIED CODE FROM TRAINING A

    #generate random sensor values for a robot placed in the center of the cell
    generated_over = 0.5 + 0.09*random.random()
    generated_under = 0.5 + 0.09*random.random()
    #right side wall
    sensor_set = ( -1000, -1000, generated_under )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( -1000, -1000, generated_over )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #left side wall
    sensor_set = ( generated_under, -1000, -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( generated_over, -1000, -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #front obstacle
    sensor_set = ( -1000, generated_under,  -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set)
    sensor_set = ( -1000, generated_under,  -1000 ) 
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set)  

    #left corner obstacle
    sensor_set = ( generated_under, generated_over,  -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( generated_over, generated_under,  -1000 ) 
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #right corner obstacle
    sensor_set = ( -1000, generated_over, generated_under )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( -1000, generated_under, generated_over )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    #dead end obstacle
    sensor_set = ( generated_under, generated_over, generated_over )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 
    sensor_set = ( generated_over, generated_under, generated_under )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #no sensors 
    sensor_set = ( -1000, -1000, -1000 )
    sensor_sets["training_data_A"][sensor_set] = get_turn_from_sensors(sensor_set) 

#test data B
for i in range(1000):
    #COPIED CODE FROM TRAINING B
    location = i%10 * 0.1 #decimeter range from sensors touching the wall to edge of the cell
    #generate random sensor values for a robot placement
    generated_num = location + 0.09*random.random()
    #right side wall 
    sensor_set = ( -1000, -1000, generated_num ) 
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set)

    #left side wall
    sensor_set = ( generated_num, -1000, -1000 )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #front obstacle
    sensor_set = ( -1000, generated_num,  -1000 ) 
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set)


    #left corner obstacle
    sensor_set = ( generated_num, generated_num,  -1000 ) 
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #right corner obstacle
    sensor_set = ( -1000, generated_num, generated_num )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #dead end obstacle
    sensor_set = ( generated_num, generated_num, generated_num )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 

    #no sensors 
    sensor_set = ( -1000, -1000, -1000 )
    sensor_sets["training_data_B"][sensor_set] = get_turn_from_sensors(sensor_set) 


this = sensor_sets["training_data_A"]
with open("training_data_A" + '.csv', mode='w') as file:
    file_writer = csv.writer(file, delimiter=',',  quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for set in this:
        row = [set[0], set[1], set[2], this[set]]
        print(row)
        file_writer.writerow(row)

this = sensor_sets["training_data_B"]
with open("training_data_B" + '.csv', mode='w') as file:
    file_writer = csv.writer(file, delimiter=',',  quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for set in this:
        row = [set[0], set[1], set[2], this[set]]
        print(row)
        file_writer.writerow(row)

this = sensor_sets["test_data_A"]
with open("test_data_A" + '.csv', mode='w') as file:
    file_writer = csv.writer(file, delimiter=',',  quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for set in this:
        row = [set[0], set[1], set[2], this[set]]
        print(row)
        file_writer.writerow(row)

this = sensor_sets["test_data_B"]
with open("test_data_B" + '.csv', mode='w') as file:
    file_writer = csv.writer(file, delimiter=',',  quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for set in this:
        row = [set[0], set[1], set[2], this[set]]
        print(row)
        file_writer.writerow(row)