"""
* image used in the conversation, is stored in images dir  
* task_interpreter_LLM gives textual and json format of the response, 
    splitting task into action and obejcts if the query is a robotic task  
* RAG is used in this code, the files are uploaded on Azure Blob storage manully, 
    then using the code in RAG dir, they are vectored into Azure AI search.
* ChatHistory is utilized into codeGenerator_LLM class to store a summary of the resonse and query
* ClipDino give the center point of the detected obejct

Steps:
    1. input the query
    2. Interpret the query into actions and objects
    3. extract obejct names
    4. give the object names to VLM
    5 put the object pose into json with its name
    6. give the whole json from the first LLM + object pose to code generator 
"""
import os, sys
import cv2
import json
from code_extractor import extract_code_from_txt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from LLMs.initial_LLM_gpt4o import task_interpreter_LLM
from LLMs.objectName_extractor import object_interpreter_LLM
from LLMs.object_position_decoder import object_position_interpreter_LLM
from LLMs.codeGenerator_LLM import codeGenerator_LLM
from VLM.image_vectorization.VLM.clipDino import ClipDino


### image is stored in image dir
images_dirpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')
image_name = os.listdir(images_dirpath)[0]
image_path = os.path.join(images_dirpath, image_name)

print("System is started ... ")

task_interpreter = task_interpreter_LLM()
objects_names_extractor = object_interpreter_LLM()
object_position_decoder = object_position_interpreter_LLM()
code_generator = codeGenerator_LLM()
object_retriever = ClipDino()

print("System is Ready.")

while True:
    objects_poses = []
    pick_place_positions = []

    ## input the query
    query = input("Input your query here. [Enter 'q' to quite.] >> ")

    ### Quit the program
    if query == 'q':
        break

    ########################################################
    ####            Interpret the Input Query           ####
    ########################################################
    text_response_inputQuery, json_response_inputQuery = task_interpreter.interpret(query)

    print("*"*10, "\n")
    print("Task interpreted: \n", text_response_inputQuery)
    print("*" * 10)

    ### json_response_inputQuery parameters: "query", "robotics_task": TRUE, "action": "pick and place", "objects": {"pick", "place"}
    is_robotic_task = json_response_inputQuery["robotics_task"]
    if is_robotic_task:
        objects_descriptions = f"{json_response_inputQuery["objects"]}"
        print(objects_descriptions)

        ########################################################
        ####            Extract the Object Names            ####
        ########################################################
        objects_names = objects_names_extractor.interpret(objects_descriptions)
        print(f"objects_names: {objects_names}")

        print("*"*10, "\n")
        print("Processing the image ... ")

        if len(objects_names) != 0:
            all_cubes_BBs = object_retriever.find_all_cubes(image_path)
            objects_poses.append({"all_cube_boundingBoxes": all_cubes_BBs})

            ### find objects in the image
            for object_name in objects_names[1:]:
                print(f"Processing the image for {object_name}... ")

                ########################################################
                ####          Find the Objects in the Image         ####
                ########################################################
                VLm_result = object_retriever.retrieve_object(object_name)
                if VLm_result is not None:
                    print("Finding th eposition of the object in the image ....")
                    object_bbox, utilized_model_inDetection = VLm_result       

                    ### structured pose of the object
                    objects_poses.append({"object_name": object_name, "object_boundingBox": object_bbox})
                else:
                    print(f"Object {object_name} is not found in the image.")
        else:
            print("No object names found in the query.")

        print(objects_poses)

        image = cv2.imread(image_path)

        ########################################################
        ####             Final Objects Positions            ####
        ########################################################
        final_positions = object_position_decoder.interpret(objects_descriptions, objects_poses)
        print(f"final_positions: {final_positions}")
        objects_descriptions = objects_descriptions.replace("'", '"')
        print(objects_descriptions)
        objects_descriptions_json = json.loads(objects_descriptions)
        if "pick" in final_positions:
            pick_object_bb = final_positions["pick"]
            pick_object_center = object_retriever.extract_centeroid(pick_object_bb)
            pick_object = {"object_description" : objects_descriptions_json["pick"], "object_location": pick_object_center}
            pick_place_positions.append(pick_object)

            pick_x, pick_y, pick_w, pick_h = pick_object_bb
            cv2.rectangle(image, (pick_x, pick_y), (pick_x + pick_w, pick_y + pick_h), (255, 255, 255), 2)
            cv2.circle(image, pick_object_center, 3, (255, 255, 255), thickness=-1)
            cv2.putText(image, "pick object", (pick_x, pick_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if "place" in final_positions:
            place_object_bb = final_positions["place"]
            place_object_center = object_retriever.extract_centeroid(place_object_bb)
            place_object = {"object_description" : objects_descriptions_json["place"], "object_location": place_object_center}
            pick_place_positions.append(place_object)

            place_x, place_y, place_w, place_h = place_object_bb
            cv2.rectangle(image, (place_x, place_y), (place_x + place_w, place_y + place_h), (255, 0, 0), 2)
            cv2.circle(image, place_object_center, 3, (255, 0, 0), thickness=-1)
            cv2.putText(image, "place object", (place_x, place_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        if len(pick_place_positions) == 0:
            print("No positions found in the query.")
        else:
            print(f"pick_place_positions: {pick_place_positions}")

        cv2.imshow("Final Object", image)
        cv2.waitKey(1)

        ########################################################
        ####     Generate Action Plan and Generate Code     ####
        ########################################################
        print("Sending data to generate Action Plan and Generate Code ....")
        ### send the object pose along with the json response from interpreter to code generator
        next_query = f"""
                        - User query:
                        {text_response_inputQuery}

                        - Objects locatiions: {pick_place_positions}
                    """

    ### the query is NOT a robotics task - send only the query
    elif not is_robotic_task:
        next_query = json_response_inputQuery["query"]
        
    final_response = code_generator.generate_answer(next_query, robotic_task= is_robotic_task, RAG_inUse=True)
    print("*"*10, "\n")
    print("Final Response: \n", final_response)
        #### add the robot coordinate changes for the object pose

    code = extract_code_from_txt(final_response)

    ########################################################
    ####                 Save the Results               ####
    ########################################################
    result_image_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "images")
    number_of_existing_files = len(os.listdir(result_image_dir))

    cv2.imwrite(os.path.join(result_image_dir, f"result_image_{number_of_existing_files}.png"), image)

    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "system_output.md"), "a") as md_file:
        md_file.write(f"**Results for Test Number {number_of_existing_files}** \n\n")
        md_file.write(f"**Query**: {query} \n\n")
        md_file.write(f"**Query Interpreted as** : \n\n`{json_response_inputQuery}` \n\n")

        if is_robotic_task:
            md_file.write(f"**Object names extracted from their Descriptions as**: `{objects_names}` \n\n")
            if VLm_result is not None:
                md_file.write(f"**Objects poisiotns found in images**: `{objects_poses}` \n\n")
                md_file.write(f"**Requested Objects poisiotns are calculated as**: `{pick_place_positions}` \n\n")
                md_file.write(f"*The image is save in `{result_image_dir}/result_image_{number_of_existing_files}.png`* \n\n")
        
        md_file.write(f"**The final resaponse of the system is:** \n\n {final_response} \n\n")
        md_file.write("-"*20)
        md_file.write("\n\n")

    print("Results are saved.")
    
cv2.destroyAllWindows()