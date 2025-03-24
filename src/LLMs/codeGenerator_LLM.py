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

    def generate_answer(self, input_query, RAG_inUse=True):
        if RAG_inUse:
            # Provide instructions to the model
            _GROUNDED_PROMPT='''
            You are an AI assistant for a UR5 robot in ROS and Gazebo simulation environment to generate codes and action plans. You will receive user queries in JSON format that may or may not be robotics tasks. If the query **is** a robotics task, you will also receive a list of JSONs containing of the object's location.

            Your objectives:
            1. If the query is **not** a robotics task, provide a brief (2–3 sentence) reliable answer.
            2. If the query **is** a robotics task:
            - Generate a step-by-step action plan.
            - Produce Python code for the UR5 robot to perform the requested actions.
            - Use the provided resources for correctness, but do not copy methods verbatim.
            - Give a list of citation of sources when they are used.
            - Refer to chat history if needed.
            3. End your response with a concise summary labeled **"history:"** for future reference.
            4. Use bullet points for multi-point answers.
            5. If you lack sufficient information, state that you do not have enough details.

            **Inputs:**
            - JSON 1 (always provided): e.g., {{
                                                "query": "pick the blue cube and place on left side of table",
                                                "robotics_task": true,
                                                "action": "pick and place",
                                                "objects": {{
                                                "pick": "the blue cube",
                                                "place": "left side of green cube"
                                                }}
                                            }}
            - List of JSONs (provided **only** if `robotics_task` is true): e.g., [{{"object_name": "the blue cube", 
                                                                                    "object_location": [x1, y1] }},
                                                                                    {{"object_name": "the green cube", 
                                                                                    "object_location": [x2, y2] }}]

            ---------------

            **Query:** {user_query}

            +++++++++++++++
            Chat_history: {history}

            Sources:
            {sources}

            '''

            # Provide the search query. 
            # The vector query finds 5 nearest neighbor matches in the search index
            # query="pick the green cube."
            _embedding = self.get_embeddings_vector(input_query)
            _vector_query = VectorizedQuery(vector=_embedding, k_nearest_neighbors=5, fields="text_vector")

            # Set up the search results and the chat thread.
            # Retrieve the selected fields from the search index related to the question.
            # Search results are limited to the top 5 matches. Limiting top can help you stay under LLM quotas.
            search_results = self._search_client.search(
                                                        search_text=input_query,
                                                        vector_queries= [_vector_query],
                                                        select=["title", "chunk"],
                                                        top=5,
                                                        )

            # Use a unique separator to make the sources distinct, such as === 
            _sources_formatted = "=================\n".join([f'TITLE: {document["title"]}, CONTENT: {document["chunk"]}' for document in search_results])
            _content = _GROUNDED_PROMPT.format(user_query=input_query, history=self._chat_history, sources=_sources_formatted)

        else:
            _GROUNDED_PROMPT = """
            You are an AI assistant for a UR5 robot in ROS and Gezebo that generates action plans and then Python codes for it to perform the robotics task requested in the query when the query is a robotic task. otherwise, provide 2-3 sentences short reliable answers to the quetions that are not a robotic_task.
            Use bullet points if the answer has multiple points.
            If you don't know the answer, say you don't have enough information.
            At the end give a very short summary to be used as a history in the chatm label "history:".
            Query: {user_query}
            """
            _content = _GROUNDED_PROMPT.format(user_query=input_query)

        response = self._openai_client.chat.completions.create(
                                                                messages=[
                                                                    {
                                                                        "role": "user",
                                                                        "content": _content
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
        llm_response = response.choices[0].message.content
        summary = self.extract_summary(llm_response)
        history = {
                    "query": input_query,
                    "summary": summary
                 }
        self._chat_history.append(history)
        # print("Chat history: \n", self._chat_history)
        return llm_response
    
    def extract_summary(self, text):
        keyword = "history"
        index = text.lower().find(keyword) 
        
        if index != -1:  # If "summary" is found
            return text[index:] 
        else:
            return None 

if __name__ == "__main__":
    llm = codeGenerator_LLM()
    while True:
        query = input("Enter query: ")
        response = llm.generate_answer(query, RAG_inUse=True)
        print(response)
        print("=====================================")