import requests
from sense_hat import SenseHat

response = requests.get('https://jsonplaceholder.typicode.com/posts')
sense = SenseHat()
sense.set_rotation(270)

if response.status_code == 200:
    data = response.json()
    print(data[0]['title'])
    sense.show_message(f'Success with code: {response.status_code}')
else:
    print(f'Failed with status code: {response.status_code}')
    sense.show_message(f'Failed with code: {response.status_code}')
