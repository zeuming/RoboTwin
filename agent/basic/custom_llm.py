from langchain_openai import ChatOpenAI,OpenAI
from langchain_aws import ChatBedrock
from basic.llm_callback import MyCustomHandler
import os
from dotenv import load_dotenv
load_dotenv()



# deepseek chat llm
deepseek_chat_llm = ChatOpenAI(
    model='deepseek-chat', 
    openai_api_key='sk-0bc806156bb04622817a392d809b92e9', 
    openai_api_base='https://api.deepseek.com',
    max_tokens=1024,
    temperature=0.0,
    verbose=True,
    callbacks= [MyCustomHandler()]
)


# claude35
claude_35_sonnet_llm = ChatBedrock(
    model_id="anthropic.claude-3-5-sonnet-20241022-v2:0",
    model_kwargs=dict(temperature=0),
    verbose=True,
    callbacks= [MyCustomHandler()]
)


if __name__ == "__main__":
    # response = deepseek_chat_llm.invoke("介绍一下黎曼猜想", temperature=1)
    # print(response.content)

    response = claude_35_sonnet_llm.invoke("介绍一下黎曼猜想", temperature=1)
    print(response.content)
