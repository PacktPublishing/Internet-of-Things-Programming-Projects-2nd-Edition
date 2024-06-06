import requests
import json

api_key = 'xxxxxxxxxxxxxxxx'
symbol = 'AAPL'

base_url = 'https://www.alphavantage.co/query?'
function = 'GLOBAL_QUOTE'

complete_url = f'{base_url}function={function} &symbol={symbol}&apikey={api_key}'

response = requests.get(complete_url)
data = response.json()

print(json.dumps(data, indent=4))
