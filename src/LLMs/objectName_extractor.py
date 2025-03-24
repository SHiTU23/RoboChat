'''
 Use GPT-4o to extract the object names from the json input
 The output is a list of object color and shapes

 EXAMPLE:
    input query: objects: {"pick": "the green cube", "place": "on top of blue cube"}
    response: 
        ["green cube", "blue cube"]
'''
#####################################

import os  
import base64
from openai import AzureOpenAI  
import json

class object_interpreter_LLM:
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
                                        You are a programming assistant that analyzes input queries to extract object colors and shapes.

                                        Instructions:
                                        You are recieving a json input with object descriptions.
                                        The desired output is a list of object color and shapes.
                                        - If spatial relations between cubes are described, set the first value in the list to 1 otherqise set it as 0.

                                        Examples of inputs and expected outputs:
                                        1.
                                        Input: 
                                        {
                                            "pick": "the blue cube",
                                            "place": "left side of green cube"
                                        }
                                        Output: 
                                        [0, "blue cube", "green cube"]

                                        2. 
                                        Input: 
                                        {
                                            "pick": "the cube on the right side of the blue cube",
                                        }
                                        Output:
                                        [1, "blue cube"]

                                        3. 
                                        Input: 
                                        {
                                            "pick": "the cube on the right side of the blue cube",
                                            "place": "on top of the cube in the right side of the green cube"
                                        }
                                        Output:
                                        [1, "blue cube", "green cube"]

                                        '''
                                        )
                        }
        
    def interpret(self, query):
        """
        This function extracts the object descriptions from the json input
        The output is a list of object names, the first value is 1 if spatial relations between cubes are described otherwise 0

        EXAMPLE:
            input:
                {
                    "pick": "the green cube", 
                    "place": "on top of blue cube"
                }
            response: 
                [0, "green cube", "blue cube"]

            Input: 
                {
                    "pick": "the cube on the right side of the blue cube",
                    "place": "on top of the cube in the right side of the green cube"
                }
                Output:
                [1, "blue cube", "green cube"]
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
    intrpreter = object_interpreter_LLM()

    input_query = '{"pick": "the green cube", "place": "on the cube located in the right side of the red cube"}'
    txt_response, json_response = intrpreter.interpret(input_query)
    print(txt_response)

    print(f"json: {json_response}")
