from twilio.rest import Client

class TwilioMessage:
    def __init__(self, account_sid, auth_token, from_number):
        """
        Initialize the TwilioNotifier with Twilio credentials.
        
        :param account_sid: str - Account SID from Twilio.
        :param auth_token: str - Auth Token from Twilio.
        :param from_number: str - The Twilio phone number for sending SMS.
        """
        self.client = Client(account_sid, auth_token)
        self.from_number = from_number

    def send_sms(self, to_number, message):
        """
        Send an SMS message using the Twilio API.
        
        :param to_number: str - The recipient's phone number including the country code.
        :param message: str - The message text to be sent.
        """
        sms = self.client.messages.create(
            body=message,
            from_=self.from_number,
            to=to_number
        )
        print(f"Message sent with SID: {sms.sid}")

# Usage
if __name__ == "__main__":
    # Replace 'your_account_sid', 'your_auth_token', and 'your_twilio_number' with your Twilio credentials
    twilio_message = TwilioMessage('-------------------------', '----------------------', '+----------------')
    twilio_message.send_sms('+-------------', 'Hello from A.R.E.S.')
