

# import MyRobot Class and math library
from fairis_tools.my_robot import MyRobot

# create robot instance
josh = MyRobot()

# load environment from maze file
maze_file = '../../worlds/Fall24/maze5.xml'
josh.load_environment(maze_file)

# Move robot to starting position
josh.move_to_start()
rotation_speed = 2.5 #m/s set to small value to ensure there is no overturning or skidding

# Main Control Loop for Robot
while josh.experiment_supervisor.step(josh.timestep) != -1:
    # get list of recognized objects from RGB camera
    obstacles = josh.rgb_camera.getRecognitionObjects()
    print(" ") #space between outputs to make more readible
    # base case - rotate until object found using PID
    if len(obstacles) == 0:
        print("No object detected yet, turning robot...")
        josh.rotate_in_place(rotation_speed)

    # if object is detected then call motion to goal method
    josh.motion_to_goal(obstacles)
    
    # taken from lab3 pdf
    # if camera has detected an object
    # if len(rec_objects) > 0:
    # extract detected object
    # landmark = rec_objects[0]
    # # object ID
    # object_id = landmark.getId()
    # # object position relative to the camera [X, Y, Z]
    # object_position = landmark.getPosition()
    # # object relative size [Y, Z]
    # object_size = landmark.getSize()
    # # object position on image [X, Y]
    # object_position_on_image = landmark.getPositionOnImage()
    # # object size on image [X, Y]
    # object_size_on_image = landmark.getSizeOnImage()
    # # object color [R, G, B]
    # object_color = landmark.getColors()
    # testing purposes
    # print("Object id:",object_id)
    # print("Object position:",object_position)
    # print("Object size:",object_size)
    # print("Object position on image:",object_position_on_image)
    # print("Object size on image:",object_size_on_image)
    # print("Object color:",object_color)
