from ollama import chat
import random


# ans = random.randint(1, 1000)
ans = int(input("input a hidden number:"))
print(ans)

def get_result(num: int) -> str:
    """Get the result of the num.

    Args:
    num: a integer num.

    Returns:
    str: if num == answer: return "YES" 
         elif: num < answer: return "Too small" 
         elif: num > answer: return "Too big"
    """
    if num == ans: return "YES"
    if num < ans: return "Too small"
    return "Too big"


messages = [{
    "role": "user", 
    "content": """
play a game to guess the hidden number from 1 to 1000. You can use the get_result function to find the answer. I aleady set a hidden number. please calling get_result to start now! and don't ask any question."""
}]

while True:
    response = chat(
        model="bonsai-2:4b", 
        messages=messages, 
        tools=[get_result], 
        think=True
    )

    messages.append(response.message)
    if response.message.tool_calls:
        for tc in response.message.tool_calls:
            if tc.function.name == "get_result":
                result = get_result(**tc.function.arguments)
                # print("THINKING:")
                # print(response.message.thinking)
                # print("CONTENT")
                print(response.message.content)
                print("%s(%d) => %s" % (tc.function.name, tc.function.arguments["num"], result))

                messages.append({
                    "role": "tool", 
                    "tool_name": tc.function.name, 
                    "content": str(result)
                })
    else: 
        break
    print("----------------------------------------------------")
    #print(messages)
    print("====================================================")
    
print("END")
for msg in messages:
    print(msg, "\n")

