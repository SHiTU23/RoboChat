'''
 This code takes a list of bounding boxes for objects with their names and a json of the object descreptions
 Returns a json of object with final bounding boxes
'''
#####################################

import os  
import base64
from openai import AzureOpenAI  
import json

class object_position_interpreter_LLM:
    def __init__(self):
        __endpoint = os.getenv("ENDPOINT_URL", "https://aihubthesiswes8755517667.openai.azure.com/")  
        self.__deployment = os.getenv("DEPLOYMENT_NAME", "gpt-4o")  
        __subscription_key = os.getenv("AZURE_OPENAI_API_KEY", "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1")  

        # Initialize Azure OpenAI Service client with key-based authentication    
        self.client = AzureOpenAI(  
            azure_endpoint=__endpoint,  
            api_key=__subscription_key,  
            api_version="2024-05-01-preview",
        )
        self.__system_message = {
                            "role": "system",
                            "content": (
                                        '''
                                        You are a programming assistant that analyzes object descriptions and their bounding boxes to return the final positions for the requested objects.
                                        Your response must be in JSON format only, with no explanations.
                                        
                                        Instructions:
                                        - You are recieving A JSON object describing pick and place instructions and A list of JSON objects, each containing of A list of all cube bounding boxes and Object names and their bounding boxes.
                                        - All the given bounidng boxes are in `[top_left_x, top_left_y, width, hight]` format.
                                        - Understand spatial relations between objects using their bounding boxes, for instance, If the user asks for the cube next to the blue cube, you must search thoufh all cubes bounding boxes and find the one that is next to the blue cube bounding box. 
                                        - If no “on top” instruction is given, make sure the placed object does not overlap with any existing object.
                                        - The position for placing the cube must not be in one of the objects bounding boxes, unless in the user query it is asked to put a cube on top of the overlaped erea. 
                                        - Return ONLY a JSON of the final bounding box as given in the example, DO NOT change its format.
                                        - DO NOT explain, just return the JSON of the requested object with its final bounding box.
                                        - ALSO for placement CHECK othe cubes to not accidently place the object on top of another objects that is not requested.

                                        Examples of inputs and expected output:
                                        List Input:
                                        [{'all_cube_boundingBoxes': [[457, 119, 37, 37], [371, 73, 31, 37], [303, 111, 32, 38], [363, 159, 32, 35], [229, 77, 31, 36]]}, {'object_name': 'blue cube', 'object_boundingBox': [363, 159, 32, 35]}, {'object_name': 'green cube', 'object_boundingBox': [371, 73, 31, 37]}]
                                        JSON Input: 
                                        {
                                            "pick": "the cube on the right side of the blue cube",
                                            "place": "on top of the cube in the right side of the green cube"
                                        }
                                        Output:
                                        {
                                            "pick": the bounding box of the cube in the right side of the green cube,
                                            "place": the bounding box of the cube in the right side of the green cube
                                        }
                                        '''
                                        )
                        }
        
    def interpret(self, object_descriptions, objects_bbs):
        query = f"""
                object_descriptions:
                {object_descriptions}

                objects_bbs:
                {objects_bbs}
                """

        user_message = {
                        "role": "user",
                        "content": query
                        }
        
        completion = self.client.chat.completions.create(  
                                                            model=self.__deployment,
                                                            messages=[self.__system_message,user_message],
                                                            max_tokens=800,  
                                                            temperature=0.4,  
                                                            top_p=0.5,  
                                                            frequency_penalty=0,  
                                                            presence_penalty=0,
                                                            stop=None,  
                                                            stream=False
                                                        )

        response = completion.to_json()
        response = json.loads(response)
        textual_answer = response["choices"][0]["message"]["content"]
        json_answer = json.loads(textual_answer)
        return json_answer


if __name__ == "__main__":
    intrpreter = object_position_interpreter_LLM()

    object_descriptions = {'pick': 'the cube on the right side of the blue cube', 'place': 'on top the cube next to green cube'}
    # object_descriptions = {'pick': 'the cube between green cube and blue cube'}
    objects_bbs = [{'all_cube_boundingBoxes': [[457, 119, 37, 37], [371, 73, 31, 37], [303, 111, 32, 38], [363, 159, 32, 35], [229, 77, 31, 36]]}, {'object_name': 'blue cube', 'object_boundingBox': [363, 159, 32, 35]}, {'object_name': 'green cube', 'object_boundingBox': [371, 73, 31, 37]}]
    json_response = intrpreter.interpret(object_descriptions, objects_bbs)

    print(f"json: {json_response}, len: {len(json_response)}")
