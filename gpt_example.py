# 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
import openai

openai.api_key = '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'

# prompt = "You are a poetic assistant, skilled in explaining complex programming concepts with creative flair"

role_system = input("Role of the system: ")
prompt = input("Prompt: ")

# Use the OpenAI API to generate a completion
completion = openai.chat.completions.create(
  model="gpt-3.5-turbo",  # specify the engine
  messages=[
    {"role": "system", "content": role_system},
    {"role": "user", "content": prompt}
  ]
)

# Print the generated text
print(completion.choices[0].message)
