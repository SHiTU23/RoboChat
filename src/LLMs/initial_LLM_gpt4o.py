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

endpoint = os.getenv("ENDPOINT_URL", "https://aihubthesiswes8755517667.openai.azure.com/")  
deployment = os.getenv("DEPLOYMENT_NAME", "gpt-4o")  
subscription_key = os.getenv("AZURE_OPENAI_API_KEY", "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1")  

input_query = input("Waht task would you like to be done by the robot? >> ")

# Initialize Azure OpenAI Service client with key-based authentication    
client = AzureOpenAI(  
    azure_endpoint=endpoint,  
    api_key=subscription_key,  
    api_version="2024-05-01-preview",
)
    


#Prepare the chat prompt 
chat_prompt = [
    {
        "role": "system",
        "content": [
            {
                "type": "text",
                "text": '''
                        You are programming assistant, split the input query into action and objects and return the reault in json format if the query was in the form of a robotic task. otherwise, return the input query as it is. 
                        output examples:  
                        if the query is not a robotic task, return {'query': 'input query', 'robotics_task': False}.
                        if input query: 'pick the blue cube'; return {'query': 'pick the blue cube', 'robotics_task': True, 'action':'pick', 'objects':{'pick':'the blue cube'}}. 
                        or input query: 'pick the blue cube and place on left side of table'; expected return: {'query': 'pick the blue cube and place on left side of table', 'robotics_task': True, 'action':'pick and place', 'objects':{'pick':'the blue cube', 'place':'left side of table'}}
                        '''
            }
        ]
    },
    {
        "role": "user",
        "content": [
            {
                "type": "text",
                # "text": "place on the highest cube."
                # "text": "pick the blue cube."
                # "text": "pick the green cube and place it 0.3 cm more to the right."
                "text": input_query
            }
        ]
    },
] 
    
# Include speech result if speech is enabled  
messages = chat_prompt  
    
print("Thinking ...")

# Generate the completion  
completion = client.chat.completions.create(  
    model=deployment,
    messages=messages,
    max_tokens=800,  
    temperature=0.5,  
    top_p=0.7,  
    frequency_penalty=0,  
    presence_penalty=0,
    stop=None,  
    stream=False
)

response = completion.to_json()
response = json.loads(response)

print("-" * 10)
print("Response is: \n",response["choices"][0]["message"]["content"])  