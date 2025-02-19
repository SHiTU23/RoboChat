### Use GPT-4o to split the query into actions and obejcts


import os  
import base64
from openai import AzureOpenAI  
import json

endpoint = os.getenv("ENDPOINT_URL", "https://aihubthesiswes8755517667.openai.azure.com/")  
deployment = os.getenv("DEPLOYMENT_NAME", "gpt-4o")  
subscription_key = os.getenv("AZURE_OPENAI_API_KEY", "16e5XT0Seh7kF6knUDWkQsLodd3otgpNR3uJuCOkyXJlVK9181MAJQQJ99BBACfhMk5XJ3w3AAAAACOGguO1")  

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
                "text": "You are programming assistant, split the input query into action and objects and return the reault in json format. examples:  if input query: 'pick the blue cube'; return 'action':'pick', 'objects':{'pick':'the blue cube'}. or input query: 'pick the blue cube and place on left side of table'; expected return: 'action':'pick and place', 'objects':{'pick':'the blue cube', 'place':'left side of table'}"
            }
        ]
    },
    {
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": "place on the highest cube."
            }
        ]
    },
    # {
    #     "role": "assistant",
    #     "content": [
    #         {
    #             "type": "text",
    #             "text": "It seems like you're referring to a specific task, such as selecting a blue cube from a set of objects. However, since I can't see or interact with physical objects, could you clarify or provide more context? For example, are you working in a virtual environment, describing a problem, or asking about coding a solution? Let me know how I can help!"
    #         }
    #     ]
    # }
] 
    
# Include speech result if speech is enabled  
messages = chat_prompt  
    
# Generate the completion  
completion = client.chat.completions.create(  
    model=deployment,
    messages=messages,
    max_tokens=800,  
    temperature=0.7,  
    top_p=0.95,  
    frequency_penalty=0,  
    presence_penalty=0,
    stop=None,  
    stream=False
)

response = completion.to_json()
response = json.loads(response)
print(response["choices"][0]["message"]["content"])  