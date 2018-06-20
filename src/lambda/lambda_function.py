import time
import os
import logging
import json
import boto3
import uuid
import urllib2

city = 'Auckland'

# Configure logger
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# Establish credentials
session_var = boto3.session.Session()
credentials = session_var.get_credentials()

# Initialize DynamoDB Client
dynamodb = boto3.client('dynamodb')

# Get weather information
def get_weather(city):
    api_base_url = "http://api.openweathermap.org/data/2.5/weather?q=%s&APPID=38a5ea89e1ee220a5c7dbdbd04c9a67d" %city
    weather = json.load(urllib2.urlopen(api_base_url))
    
    return weather['weather'][0]['main']

def welcome_handler(intent):
    instruction = "Welcome to Robotics lab."
    weather = get_weather(city)
    
    if(weather is not 'Clear'):
        instruction += "Despite the bad weather, thank you so much for comming here. "
    else:
        instruction += "It is so nice weather for looking around our lab. isn't it? "
    instruction += "You can ask anything about our lab."

    return close(intent['sessionAttributes'],
                'Fulfilled',
                {'contentType': 'PlainText',
                'content': instruction})






""" =========================================================================================================================== """
""" =========================== Generic functions used to simplify interaction with Amazon Lex ================================ """

def get_slots(intent_request):
    return intent_request['currentIntent']['slots']

def elicit_slot(session_attributes, intent_name, slots, slot_to_elicit, message):
    return {
        'sessionAttributes': session_attributes,
        'dialogAction': {
            'type': 'ElicitSlot',
            'intentName': intent_name,
            'slots': slots,
            'slotToElicit': slot_to_elicit,
            'message': message
        }
    }

def close(session_attributes, fulfillment_state, message):
    response = {
        'sessionAttributes': session_attributes,
        'dialogAction': {
            'type': 'Close',
            'fulfillmentState': fulfillment_state,
            'message': message
        }
    }
    return response

def delegate(session_attributes, slots):
    return {
        'sessionAttributes': session_attributes,
        'dialogAction': {
            'type': 'Delegate',
            'slots': slots
        }
    }

def build_validation_result(is_valid, violated_slot, message_content):
    if message_content is None:
        return {
            "isValid": is_valid,
            "violatedSlot": violated_slot,
        }
    return {
        'isValid': is_valid,
        'violatedSlot': violated_slot,
        'message': {'contentType': 'PlainText', 'content': message_content}
    }

def dispatch(intent_request):
    logger.debug('dispatch userId={}, intentName={}'.format(intent_request['userId'], intent_request['currentIntent']['name']))
    intent_name = intent_request['currentIntent']['name']

    if intent_name == 'welcome':
        return welcome_handler(intent_request)

    raise Exception('Intent with name ' + intent_name + ' not supported')

def lambda_handler(event, context):
    os.environ['TZ'] = 'America/Los_Angeles'
    time.tzset()
    logger.info('Received event: {}'.format(event))

    return dispatch(event)