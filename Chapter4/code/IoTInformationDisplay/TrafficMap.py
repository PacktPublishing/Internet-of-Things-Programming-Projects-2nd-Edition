import requests


class TrafficMap:

    def __init__(self, latitude, longitude, zoom):
        self.latitude = latitude
        self.longitude = longitude
        self.zoom = zoom
        self.size = "500,325"
        self.api_key = "i6HteDYdSylP4rfJr8JdLxCYpnbPKSeu"

    def get_traffic_map(self):
        base_url = "http://www.mapquestapi.com/staticmap/v5/map"

        # Define parameters
        params = {
            'key': self.api_key,
            'center': f"{self.latitude},{self.longitude}",
            'zoom': self.zoom,
            'size': self.size,
            'traffic': 'flow|cons|inc'
        }

        # Make the API call
        response = requests.get(base_url, params=params)

        # Check if the request was successful
        if response.status_code == 200:
            # Save the map as an image file
            with open('images/traffic_map.png', 'wb') as f:
                f.write(response.content)
            return "images/traffic_map.png"
        else:
            return "images/blank.png"


if __name__ == "__main__":
    latitude = 43.6426
    longitude = -79.3871
    traffic_map = TrafficMap(latitude, longitude, 12)
    print(traffic_map.get_traffic_map())
