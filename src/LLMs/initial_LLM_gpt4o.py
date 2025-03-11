'''
 Use GPT-4o to split the query into actions and obejcts
 The input is a query that contains a pick and place task to be done by the robot.
 The output is a json file with action and objects

 EXAMPLE:
    input query: pick the green cube and place it in position [10, 20].
    response: 
        {
        "query": "pick the green cube and place it in position [10, 20].",
        "robotics_task": TRUE,
        "action": "pick and place",
        "objects": {
                    "pick": "the green cube",
                    "place": "position [10, 20]"
                    }
        }
'''
#####################################

import os  
import base64
from openai import AzureOpenAI  
import json

class task_interpreter:
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
                                        You are a programming assistant that analyzes input queries to determine if they represent a robotic task. Your response should be in JSON format.

                                        Instructions:
                                        - If the input query does not represent a robotic task, simply return the query as is along with an indicator that it is not a robotics task.
                                        - If the input query describes a robotic task, split it into its action and objects and return these details in JSON format.

                                        Output Examples:
                                        1. For a non-robotic query:
                                        Input: "What is the weather today?"
                                        Output: 
                                        {
                                            "query": "What is the weather today?",
                                            "robotics_task": false
                                        }

                                        2. For a single-action robotic task:
                                        Input: "pick the blue cube"
                                        Output:
                                        {
                                            "query": "pick the blue cube",
                                            "robotics_task": true,
                                            "action": "pick",
                                            "objects": {
                                            "pick": "the blue cube"
                                            }
                                        }

                                        3. For a multi-action robotic task:
                                        Input: "pick the blue cube and place on left side of table"
                                        Output:
                                        {
                                            "query": "pick the blue cube and place on left side of table",
                                            "robotics_task": true,
                                            "action": "pick and place",
                                            "objects": {
                                            "pick": "the blue cube",
                                            "place": "left side of table"
                                            }
                                        }

                                        '''
                                        )
                        }
        
        print("System is now Ready.")

    def interpret(self, query):
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
        return response["choices"][0]["message"]["content"]

            


# "text": "place on the highest cube."
# "text": "pick the blue cube."
# "text": "pick the green cube and place it 0.3 cm more to the right."


if __name__ == "__main__":
    intrpreter = task_interpreter()
    while True:
        input_query = input("Waht task would you like to be done by the robot? >> ")
        response = intrpreter.interpret(input_query)
        r = json.loads(response)
        print(r)

        print("type: ", type(r))
        print("-"*20)