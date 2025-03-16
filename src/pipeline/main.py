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
    2. give the pick object name to VLM
    3. put the object pose into json with its name
    4. give the whole json from the first LLM + object pose to code generator 
"""
import os, sys
import cv2
from code_extractor import extract_code_from_txt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from LLMs.initial_LLM_gpt4o import task_interpreter_LLM
from LLMs.codeGenerator_LLM import codeGenerator_LLM
from VLM.image_vectorization.VLM.clipDino import ClipDino


### image is stored in image dir
images_dirpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')
image_name = os.listdir(images_dirpath)[0]
image_path = os.path.join(images_dirpath, image_name)

print("System is started ... ")

task_interpreter = task_interpreter_LLM()
code_generator = codeGenerator_LLM()
object_retriever = ClipDino()

print("System is Ready.")

while True:
    key = cv2.waitKey(1) & 0xFF

    ## input the query
    query = input("Input your query here. [Enter 'q' to quite.] >> ")

    if query == 'q':
        break

    text_response, json_response = task_interpreter.interpret(query)

    print("*"*10, "\n")
    print("Task interpreted: \n", text_response)
    print("*" * 10)

    ### json_response parameters: "query", "robotics_task": TRUE, "action": "pick and place", "objects": {"pick", "place"}
    is_robotic_task = json_response["robotics_task"]
    if is_robotic_task:
        object2pick_name = json_response["objects"]["pick"]
        print("object to pick :", object2pick_name)

        print("*"*10, "\n")
        print("Processing the image ... ")

        ### give the name of the requested object to the VLM
        VLm_result = object_retriever.retrieve_object(image_path, object2pick_name)
        if VLm_result is not None:
            print("Finding th eposition of the object in the image ....")
            object_bbox, utilized_model_inDetection = VLm_result       
            obejct_center_point = object_retriever.extract_centeroid()

            ### structured pose of the object
            object_pose = {"object_name": object2pick_name, "object_location": obejct_center_point}
            print(object_pose)
            
            retrieved_obejct_image = object_retriever.image_retrivedObject()
            cv2.imshow("retrieved_obejct_image", retrieved_obejct_image)
            cv2.waitKey(1)

            print("Sending data to generate Action Plan and Generate Code ....")
            ### send the object pose along with the json response from interpreter to code generator
            next_query = f"""
                            JSON 1:
                            {json_response}
                            -------------------
                            JSON 2:
                            {object_pose}
                        """

    ### the query is NOT a robotics task
    else:
        next_query = f"""
                            JSON 1:
                            {json_response}
                        """
        
    final_response = code_generator.generate_answer(next_query, RAG_inUse=True)
    print("*"*10, "\n")
    print("Final Response: \n", final_response)
        #### add the robot coordinate changes for the object pose

    code = extract_code_from_txt(final_response)
    
cv2.destroyAllWindows()