### Use GPT-4o and Azure AI Search for RAG to generate codes for ur5 robot in ROS and python 

from azure.search.documents import SearchClient
from openai import AzureOpenAI  
from azure.core.credentials import AzureKeyCredential
from azure.search.documents.models import VectorizedQuery
import os  
import json



class codeGenerator_LLM:
    def __init__(self):
        self._AZURE_OPENAI_ENDPOINT = "https://aihubthesiswes8755517667.openai.azure.com/"
        self._AZURE_CHAT_MODELNAME = "gpt-4o-codeGenerator"
        self._AZURE_EMBEDDING_MODEL = "text-embedding-3-large"

        self._AZURE_SEARCH_SERVICE = "https://thesis-west-aisearch.search.windows.net"
        self._AZURE_SEARCH_ADMIN_KEY = "RyHN2fSd1HrLqlwpfUuTbVYwmTO4W9rR3OxO9FZko6AzSeDsPllI"

        _index_name = "robotics-docs-index"

        _credential = AzureKeyCredential(self._AZURE_SEARCH_ADMIN_KEY)
        self._openai_client = AzureOpenAI(
                                            api_version="2024-06-01",
                                            azure_endpoint=self._AZURE_OPENAI_ENDPOINT,
                                         )

        self._search_client = SearchClient(
                                            endpoint=self._AZURE_SEARCH_SERVICE,
                                            index_name=_index_name,
                                            credential=_credential
                                          )

        self._chat_history = []
    def get_embeddings_vector(self, text):
        response = self._openai_client.embeddings.create(
                                                            input=text,
                                                            model=self._AZURE_EMBEDDING_MODEL,
                                                            dimensions=1024,
                                                        )
        embedding = response.data[0].embedding
        return embedding

    def generate_answer(self, input_query, robotic_task, RAG_inUse=True):
        if RAG_inUse:
            if robotic_task:
                ### first generate an action plan and then search through the provided documents to find the needed functions
                _ACTIONPLAN_GROUNDED_PROMPT = """
                You are an AI assistant for a UR5 robot in ROS and Gazebo simulation environment to generate a detail action plans for completing the requested task. You will receive user queries in JSON format that includes the requested robotic task, and receive a list of JSONs containing of the object's location. 
                
                Your objectives:
                - Generate a detailed step-by-step action plan for performing the task by a ur-5 robot in ros.
                - Use bullet points for multi-point answers.
                - The action plan should be very detailed and including all the steps for the task in the simulation.
                - You must refer to the provided sources for the correctness of the steps and not missing the important steps.
                - Extract exact functions names from the sources that are needed for the task to be complete for each step. 
                - Refer to the chat history if needed.
                - If you lack sufficient information, state that you do not have enough details.

                **Example of Inputs:**
                - User query: 
                {{
                    "query": "pick the green cube",
                    "robotics_task": true,
                    "action": "pick",
                    "objects": {{
                    "pick": "the green cube",
                    }}
                }}

                - Objects locatiions: [{{'object_description': 'the green cube', 'object_location': (399, 128)}}]

                ----------------
                **Inputs:** 
                {user_query}

                **Sources:**
                {sources}
                """
                # **Chat_history:**
                # {history}

                Search_query_for_actionPlan = f"""
                                query: {input_query}
                                Extract all the functions names and steps for compeleting the requied task given in 'query' by the robot in simulation. Do not miss steps.
                                """


                _actionPlan_embedding = self.get_embeddings_vector(Search_query_for_actionPlan)
                _actionPlan_vector_query = VectorizedQuery(vector=_actionPlan_embedding, k_nearest_neighbors=10, fields="text_vector")

                print("Searching for the relevant docs to the query for generating action plan")
                search_results_for_actionPlan = self._search_client.search(
                                                            search_text=Search_query_for_actionPlan,
                                                            vector_queries= [_actionPlan_vector_query],
                                                            select=["title", "chunk"],
                                                            top=20, # number of documents to return
                                                            )

                # Use a unique separator to make the sources distinct, such as === 
                _formatted_sources_for_actionPlan = "=================\n".join([f'TITLE: {document["title"]}, CONTENT: {document["chunk"]}' for document in search_results_for_actionPlan])
                # _content_for_actionPlan = _ACTIONPLAN_GROUNDED_PROMPT.format(user_query=input_query, history=self._chat_history, sources=_formatted_sources_for_actionPlan)
                _content_for_actionPlan = _ACTIONPLAN_GROUNDED_PROMPT.format(user_query=input_query, sources=_formatted_sources_for_actionPlan)

                print("Waiting for generating Action Plan")
                #### send to the LLM to generate the action plan
                actionPlan_response = self._openai_client.chat.completions.create(
                                                                                    messages=[
                                                                                        {
                                                                                            "role": "user",
                                                                                            "content": _content_for_actionPlan
                                                                                        }
                                                                                    ],
                                                                                    model=self._AZURE_CHAT_MODELNAME,
                                                                                    max_tokens=5000,  
                                                                                    temperature=0.5,  
                                                                                    top_p=0.6,  
                                                                                    frequency_penalty=0,  
                                                                                    presence_penalty=0,
                                                                                    stop=None,  
                                                                                    stream=False
                                                                                )
                llm_actionPlan_response = actionPlan_response.choices[0].message.content
                print("Action Plan: \n", llm_actionPlan_response)

                #### Now the Action Plan goes to the RAG to find the needed functions for the code and generate the final code

                # - Strictly reuse full function definitions from the provided sources. Only replace function arguments (e.g., object name, x, y) with the values given in the input. Do NOT rewrite or restructure code unless explicitly required.
                # - Note the collision object to add it correctly and as it is structured in the sample codes.
                # - Do NOT change anything in the code sctructure unless explicitly instructed. Import all the libraries in the code. 
                # - Copy codes from the code samples in the provided sources and give an accurate and executable Python code for the UR5 robot to perform the requested actions. Do not miss packages and libraries to include in the code correctly.
                # Based on the given action plan and searching though the provided sources to find the corresponding code blocks for the action plan and generate a code that includes all the necessary libraries, functions and code steps from the code samples and provided documents.
# Your objectives:
#                 - The user has all the sample codes in the provided sources, so you need to write a code that uses those functions and classes from the provided sources to perform the task. 
#                 - Frist Refer to the Action Plan that is given in the following and consider the task requested in the Query. 
#                 - Search through the provided Sources for the functions and classes that are needed to perform the task.
#                 - Do not write the bodies of functions just call them in your generated code.
#                 - Write a Python code to perform the task by calling the correct functions from the provided sources and supplementing the correct values.
#                 - Import all the necessary libraries and packages in the code.
#                 - Note that the usedr might ask to place a cube "on top" of another, then you should caculate the correct z coordinate for the cube to be placed on top of the other cube.
#                 - Give a list of citation of sources when you used them.
#                 - Refer to chat history if needed.
#                 - End your response with a concise summary labeled **"history:"** for future reference.
#                 - If you lack sufficient information, state that you do not have enough details.
#                 - Be careful with generating the code, you should consider everything and everything should work properly.

                # You are an AI assistant for a UR5 robot in ROS and Gazebo simulation environment to generate accurate and executable Python code for the robot to perfrom the requested task in  Gazebo simulation controlled by ROS. You will receive user queries in JSON format that includes the requested robotic task, and receive a list of JSONs containing of the object's location, and the generated Action Plan for the task accomplishment.


                _CODE_GENERATOR_GROUNDED_PROMPT='''
                === STYLE GUARDRAIL ===
                • Your output MUST ONLY contain:
                    1. import statements
                    2. calls to existing functions/classes with literal argument values
                • You CANNOT include:
                    – Any function or class definitions
                    – Any code excerpted from the bodies of functions

                === INSTRUCTIONS ===
                Your task:  
                Generate a Python script for a UR5 robot by calling pre-existing functions and classes from the provided source files—do NOT rewrite or modify any function bodies.

                1. Inputs:  
                - **Action Plan**: an ordered list of function names (and any parameter placeholders).  
                - **Source Files**: code samples containing full definitions of those functions and classes.

                2. Procedure:  
                a. Read the Action Plan and the user’s query.  
                b. Locate each function or class in the Source Files.  
                c. Calculate any derived parameters (e.g., z-coordinate when stacking cubes).  
                d. **Do not** write or change function bodies—only call them.  
                e. Import all required libraries/modules exactly as they appear in the sources, Import the script name if you are calling a method from it.

                3. Output:  
                - A single Python script that:  
                    1. Imports necessary packages.  
                    2. Calls each function in the order given, supplying the correct argument values.  
                - A **Citations** section listing which source file or sample provided each function.  
                - A one-paragraph **history:** summary of what was generated and why.

                4. Missing Information:  
                - If any function or parameter value is unavailable or ambiguous, respond with:  
                    `Insufficient information: [describe what’s missing].`


                **Example of Inputs:**
                - User query: 
                {{
                    "query": "pick the green cube",
                    "robotics_task": true,
                    "action": "pick",
                    "objects": {{
                    "pick": "the green cube",
                    }}
                }}

                - Objects locatiions: [{{'object_description': 'the green cube', 'object_location': (399, 128)}}]

                ---------------
                **Inputs:** 
                {user_query}

                **Action Plan:**
                {action_plan}

                **Provided Sources:**
                {sources}

                '''

                ### Search through the docs for functions and code
                Search_query_for_codeGeneration = f"""
                                Input Query: {input_query}
                                List all the functions and classes necessary for the code to compelete the requied task given in 'Input Query' by the robot in simulation. Do not miss any part in the function bodies.
                                """


                _codeGeneration_embedding = self.get_embeddings_vector(Search_query_for_codeGeneration)
                _codeGeneration_vector_query = VectorizedQuery(vector=_codeGeneration_embedding, k_nearest_neighbors=10, fields="text_vector")

                search_results_for_codeGeneration = self._search_client.search(
                                                            search_text=Search_query_for_codeGeneration,
                                                            vector_queries= [_codeGeneration_vector_query],
                                                            select=["title", "chunk"],
                                                            top=20, # number of documents to return
                                                            )

                # Use a unique separator to make the sources distinct, such as === 
                _codeGeneration_sources_formatted = "=================\n".join([f'TITLE: {document["title"]}, CONTENT: {document["chunk"]}' for document in search_results_for_codeGeneration])
                _final_content = _CODE_GENERATOR_GROUNDED_PROMPT.format(user_query=input_query, action_plan=llm_actionPlan_response, sources=_codeGeneration_sources_formatted)

            elif not robotic_task:
                _GROUNDED_PROMPT = """
                You are an AI assistant for a UR5 robot in ROS and Gezebo that generates action plans and then Python codes for it to perform the robotics task requested in the query when the query is a robotic task. Now that the query is not a robotic task, you should provide 2-3 sentences short reliable answers to the quetions.
                
                Your objectives:
                - Use bullet points if the answer has multiple points.
                - If you don't know the answer, say you don't have enough information.
                - End your response with a concise summary labeled **"history:"** for future reference.

                ---------------
                **Query:**
                {user_query}

                **Chat_history:**
                {history}
                """
                _final_content = _GROUNDED_PROMPT.format(user_query=input_query, history=self._chat_history,)

        elif not RAG_inUse:
            _GROUNDED_PROMPT = """
            You are an AI assistant for a UR5 robot in ROS and Gezebo that generates action plans and then Python codes for it to perform the robotics task requested in the query when the query is a robotic task. otherwise, provide 2-3 sentences short reliable answers to the quetions that are not a robotic_task.
            
            Your objectives:
            - Use bullet points if the answer has multiple points.
            - If you don't know the answer, say you don't have enough information.
            - End your response with a concise summary labeled **"history:"** for future reference.

            ---------------
            **Query:**
            {user_query}

            **Chat_history:**
            {history}
            """
            _final_content = _GROUNDED_PROMPT.format(user_query=input_query, history=self._chat_history,)


        final_response = self._openai_client.chat.completions.create(
                                                                messages=[
                                                                    {
                                                                        "role": "user",
                                                                        "content": _final_content
                                                                    }
                                                                ],
                                                                model=self._AZURE_CHAT_MODELNAME,
                                                                max_tokens=5000,  
                                                                temperature=0.5,  
                                                                top_p=0.6,  
                                                                frequency_penalty=0,  
                                                                presence_penalty=0,
                                                                stop=None,  
                                                                stream=False
                                                            )
        llm_final_response = final_response.choices[0].message.content

        summary = self.extract_summary(llm_final_response)
        history = {
                    "query": input_query,
                    "summary": summary
                 }
        self._chat_history.append(history)

        final_report = f"""
                    response for Action plan: \n
                    {llm_actionPlan_response}
                    \n\n
                    response for code generation: \n
                    {llm_final_response}
                    """
        return final_report
    
    def extract_summary(self, text):
        keyword = "history"
        index = text.lower().find(keyword) 
        
        if index != -1:  # If "summary" is found
            return text[index:] 
        else:
            return None 

if __name__ == "__main__":
    llm = codeGenerator_LLM()

    user_query = '''
            {
                "query": "pick the cube on the right side of the blue cube and place it on top the cube next to green cube",
                "robotics_task": true,
                "action": "pick and place",
                "objects": {
                    "pick": "the cube on the right side of the blue cube",
                    "place": "the cube next to green cube"
                }
            }
            '''
    object_locations = [{'object_description': 'the green cube', 'object_location': (399, 128)}]
    
    query = f"""
                - User query:
                {user_query}

                - Objects locatiions: {object_locations}
            """
    
    response = llm.generate_answer(query, robotic_task=True, RAG_inUse=True)
    print(response)
    print("=====================================")