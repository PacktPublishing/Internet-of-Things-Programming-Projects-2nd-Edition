import requests
from sense_hat import SenseHat

response = requests.get(
    'https://jsonplaceholder.typicode.com/posts'
)
sense = SenseHat()
sense.set_rotation(270)

if response.status_code == 200:
    data = response.json()
    print(data[0]['title'])
    
    success_msg = 'Success with code: '
    success_msg += str(response.status_code)
    sense.show_message(success_msg)
else:
    error_msg = 'Failed with code: '
    error_msg += str(response.status_code)
    print(error_msg)
    sense.show_message(error_msg)
