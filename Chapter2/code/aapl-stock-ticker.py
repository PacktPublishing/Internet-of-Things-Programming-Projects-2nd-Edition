import requests
import json
from sense_hat import SenseHat
import time

api_key = 'xxxxxxxxxxxxxxxx'
symbol = 'AAPL'

base_url = 'https://www.alphavantage.co/query?'
function = 'GLOBAL_QUOTE'

sense = SenseHat()
sense.set_rotation(270)

last_call_time = time.time() - 180  # Set initial time to allow immediate first call

last_ticker_info = ""

while True:
    current_time = time.time()

    if current_time - last_call_time >= 180:
        complete_url = f'{base_url}function={function}&symbol={symbol}&apikey={api_key}'

        response = requests.get(complete_url)
        data = response.json()

        quote = data['Global Quote']
        ticker_info = f"{quote['01. symbol']} \
        Price: {quote['05. price']} \
        Volume: {quote['06. volume'] } \
        Change: {quote['09. change'] }"

        last_ticker_info = ticker_info

        sense.show_message(ticker_info, scroll_speed=0.05, text_colour=[255, 255, 255])
        last_call_time = current_time

    else:
        sense.show_message(last_ticker_info, scroll_speed=0.05, text_colour=[255, 255, 255])

    time.sleep(1)
