import cv2, random, rospkg, os
import numpy as np
import matplotlib.backends.backend_pdf as pdf
import matplotlib.pyplot as plt

from PIL import Image

PAGE_DPI = 200
MARKER_SIZE = 450
DICT_STR = 'cv2.aruco.DICT_5X5_250'
DICT = cv2.aruco.DICT_5X5_250
# Function to generate ArUco marker image
def generate_aruco_marker(marker_id, marker_size):
    # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
    return cv2.aruco.generateImageMarker(dictionary=aruco_dict,id=marker_id,sidePixels=marker_size)

def transform_pixel_board_mm(marker, c_x, c_y, pixel_mm):
    corners_marker_transformed = marker - np.array([c_x, c_y])
    corners_marker_transformed[:,1] = corners_marker_transformed[:,1] * -1
    return corners_marker_transformed*pixel_mm/1000

# Function to create a board with two ArUco markers placed horizontally
def generate_aruco_board(id_0, id_1):
    # Define parameters
    marker_size = MARKER_SIZE
    space_between_markers = 50
    board_width = marker_size * 2 + space_between_markers
    board_height = marker_size
    
    # Create a blank white image (A4 size: 8.27 x 11.69 inches at 300 dpi)[ROTATED]
    board_image = np.ones((int(8.27 * PAGE_DPI), int(11.69 * PAGE_DPI)), dtype=np.uint8) * 255

    x_offset = (board_image.shape[1] - board_width)//2
    y_offset = (board_image.shape[0] - board_height)//2
    print(board_image.shape, x_offset)

    # Generate ArUco markers
    marker1 = generate_aruco_marker(id_0, marker_size)
    marker2 = generate_aruco_marker(id_1, marker_size)

    # # Place markers on the board
    board_image[y_offset:y_offset + marker_size, x_offset:x_offset + marker_size] = marker1
    board_image[y_offset:y_offset + marker_size, x_offset + marker_size + space_between_markers:x_offset + 2 * marker_size + space_between_markers] = marker2
    # cv2.rectangle(board_image, (y_offset-20, x_offset-30), (y_offset + board_width+30, x_offset + board_height+25), color=(125), thickness=4)
    cv2.rectangle(board_image, (x_offset - 10, y_offset - 10), 
                        (x_offset + 2 * marker_size + space_between_markers + 10,
                        y_offset + marker_size + 10), color=(125), thickness=4)
    # Add text at the top middle
    text = f"NCS Greenhouse ArUco Marker Board, ID:{id_0//2}"
    font = cv2.FONT_HERSHEY_COMPLEX
    font_scale = 3
    font_thickness = 2
    text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
    text_position = ((board_image.shape[1] - text_size[0]) // 2, board_image.shape[0]//8)
    cv2.putText(board_image, text, text_position, font, font_scale, (0, 0, 0), font_thickness, cv2.LINE_AA)

    # Add an arrow from the top middle to the bottom middle
    arrow_start = (x_offset+ marker_size//2 ,y_offset + marker_size + board_image.shape[0]//40)
    arrow_end = (x_offset + marker_size//2 - board_image.shape[1]//20 , y_offset + marker_size + 100 + board_image.shape[0]//10)
    arrow_color = (0)  # BGR color

    cv2.arrowedLine(board_image, arrow_start, arrow_end, arrow_color, thickness=3, tipLength=0.1)
    text_size = cv2.getTextSize(f"ID:{id_0}", font, font_scale, font_thickness)[0]
    text_position = (arrow_end[0] - text_size[0],arrow_end[1]+text_size[1])
    cv2.putText(board_image, f"ID:{id_0}", text_position, font, font_scale, (0, 0, 0), font_thickness, cv2.LINE_AA)
    
    # Add an arrow from the top middle to the bottom middle
    arrow_start = (x_offset+ marker_size//2 + marker_size + space_between_markers ,y_offset + marker_size + board_image.shape[0]//40)
    arrow_end = (x_offset+ marker_size//2 + marker_size + space_between_markers +  board_image.shape[1]//20 , y_offset + marker_size + 100 + board_image.shape[0]//10)
    arrow_color = (0)  # BGR color

    cv2.arrowedLine(board_image, arrow_start, arrow_end, arrow_color, thickness=3, tipLength=0.1)
    text_size = cv2.getTextSize(f"ID:{id_1}", font, font_scale, font_thickness)[0]
    text_position = (arrow_end[0],arrow_end[1]+text_size[1])
    cv2.putText(board_image, f"ID:{id_1}", text_position, font, font_scale, (0, 0, 0), font_thickness, cv2.LINE_AA)


    text_size = cv2.getTextSize(f"automatically generated by aruco_board_generator.py", font, font_scale, font_thickness)[0]
    text_position = (text_size[1], board_image.shape[0] - text_size[1]*2)
    cv2.putText(board_image, f"automatically generated by aruco_board_generator.py", text_position, font, 1, (0, 0, 0), font_thickness, cv2.LINE_AA)
    
    text_size = cv2.getTextSize(f"Dict: {DICT_STR}", font, font_scale, font_thickness)[0]
    text_position = (text_size[1], board_image.shape[0] - text_size[1])
    cv2.putText(board_image, f"Dict: {DICT_STR}", text_position, font, 1, (0, 0, 0), font_thickness, cv2.LINE_AA)


    # Define the physical size in millimeters # A4: 210*297
    
    pixel_size_mm = 210/ board_image.shape[0]
    center_x = board_image.shape[1] // 2
    center_y = board_image.shape[0] // 2
    corners_marker1 = np.array([[x_offset, y_offset], [x_offset + marker_size, y_offset], [x_offset + marker_size, y_offset + marker_size], [x_offset, y_offset + marker_size]], dtype=np.float32)
    corners_marker1_transformed = transform_pixel_board_mm(corners_marker1,center_x,center_y,pixel_size_mm)
    corners_marker2 = np.array([[x_offset + space_between_markers + marker_size, y_offset], [x_offset + 2 * marker_size + space_between_markers, y_offset], [x_offset + 2 * marker_size + space_between_markers, y_offset + marker_size], [x_offset + space_between_markers + marker_size, y_offset + marker_size]], dtype=np.float32)
    corners_marker2_transformed = transform_pixel_board_mm(corners_marker2,center_x,center_y,pixel_size_mm)

    new_column = np.zeros((corners_marker1.shape[0], 1))
    corners_marker1_transformed = np.hstack((corners_marker1_transformed,new_column))
    corners_marker2_transformed = np.hstack((corners_marker2_transformed,new_column))

    marker1_str = np.array2string(corners_marker1_transformed, separator=', ', precision=4).replace('\n','')
    marker2_str = np.array2string(corners_marker2_transformed, separator=', ', precision=4).replace('\n','')
    python_prompt = [   f"board_corners_{id_0//2} = [np.array({marker1_str},dtype=np.float32), np.array({marker2_str},dtype=np.float32),]",
                        f"board_ids_{id_0//2}=np.array([[{id_0}],[{id_1}]], dtype = np.int32)"]
    return board_image, python_prompt


def create_folder_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"Folder '{folder_path}' created.")
    else:
        print(f"Folder '{folder_path}' already exists.")

num_marker_gen = 10
dict_path:str = rospkg.RosPack().get_path('robot_control_pkg')
dict_path = dict_path.replace('src/robot_control_pkg','aruco_boards/')
create_folder_if_not_exists(dict_path)

python_prompts = []
for i in range(num_marker_gen):
    board_img, python_prompt = generate_aruco_board(2*i,2*i+1)
    output_pdf_path = dict_path + f'Board_ID_{i}.pdf'
    pli_image = Image.fromarray(cv2.cvtColor(board_img,cv2.COLOR_BGR2RGB))
    pli_image.save(output_pdf_path)
    python_prompts.append(python_prompt)


#generating a python class
pyfile_path = rospkg.RosPack().get_path('robot_control_pkg') + "/scripts/utils/_aruco_boards.py"
command_lines = [   '# This script is automatically generated. \n# Modifying this code is not recommanded\n',
                    'import cv2\n',
                    'import numpy as np\n']
with open(pyfile_path, 'w') as file:
    file.writelines(command_lines)
    file.write('def get_boards():\n')
    file.write('\treturn boards\n')
    file.write('\n')
    file.write('def get_dict():\n')
    file.write('\treturn aruco_dict\n')
    file.write('\n')
    file.write(f'aruco_dict=cv2.aruco.getPredefinedDictionary({DICT_STR})\n')
    for line in python_prompts:
        file.write(line[0]+'\n')
    file.write('\n')
    for line in python_prompts:
        file.write(line[1]+'\n')
    file.write('\n')
    for i in range(num_marker_gen):
        file.write(f'board_{i} = cv2.aruco.Board(board_corners_{i},aruco_dict,board_ids_{i})\n')

    file.write('boards = []\n')
    for i in range(num_marker_gen):
        file.write(f'boards.append(board_{i})\n')
    
    file.write('\n')
