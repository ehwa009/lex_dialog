import time
import os
import logging
import json
import boto3
import uuid
import urllib2
import random

from boto3.dynamodb.conditions import Key, Attr

city = 'Auckland'

# Configure logger
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# Establish credentials
session_var = boto3.session.Session()
credentials = session_var.get_credentials()

# Initialize DynamoDB Client
member_table = 'uoaRobotics'
dynamodb = boto3.client('dynamodb')

WEATHER_TEXT = [
    "The weather in {city} now is {current_weather_desc}, current temperature is {current_temp} degree and wind speed is {current_wind_speed} m/s.",
]

# Get weather information
def get_weather(city):
    api_base_url = "http://api.openweathermap.org/data/2.5/weather?q=%s&APPID=38a5ea89e1ee220a5c7dbdbd04c9a67d" %city
    result = json.load(urllib2.urlopen(api_base_url))
    
    return result

def welcome_handler(intent):
    instruction = "<expression=happiness:1.0>Hello! everyone? <br=1> Welcome to the Center for Automation and Robotic Engineering Science. <br=1> \n"
    instruction += "We are an interdisciplinary research hub. <br=0> \n"
    instruction += "My name is EveR. I am an android robot.  <br=1> \n"
    weather_data = get_weather(city)
    weather = weather_data['weather'][0]['main']
    
    if(weather is not 'Clear'):
        # instruction += "Despite the bad weather, thank you so much for comming here. "
        instruction += "It is so nice weather for looking around our lab. isn't it? <br=1> \n"
    else:
        instruction += "It is so nice weather for looking around our lab. isn't it? <br=1> \n"
    instruction += "You can ask me about weather, our lab and face expression. <br=1>"

    return close(intent['sessionAttributes'],
                'Fulfilled',
                {'contentType': 'PlainText',
                'content': instruction})

def intro_staff_handler(intent):
    source = intent['invocationSource']
    slots = get_slots(intent)
    if(source == 'DialogCodeHook'):
        return elicit_slot(
            intent['sessionAttributes'],
            intent['currentIntent']['name'],
            slots, None, None)
    member_detail = dynamodb.get_item(
        TableName = member_table,
        Key = {
            'Name': {
                'S': 'bruce'
            }
        }
    )
    return close(intent['sessionAttributes'],
                 'Fulfilled',
                 {'contentType': 'PlainText',
                 'content': 'Ok, here is information of %s. %s' %(slots['staff'], member_detail['Item']['desc']['S'])}
                 )

def intro_lab_handler(intent):
    say = '''<expression=happiness:1.0>alright. <br=1>
        CARES evolved from the robotics research activity across the University of Auckland. <br=1> \n
        An interdisciplinary group developed over time <br=1> \n
        and then came together in 2010 with strategic funding from the Vice Chancellor for three years to enhance international research activity in robotics. <br=1> \n
    '''

    return close(intent['sessionAttributes'],
                'Fulfilled',
                {'contentType': 'PlainText',
                'content': say})


def demo_face_expression(intent):
    say = "<expression=happiness:1.0>Ok. <br=1>"
    say += '''I have many facial expressions. It is helpful to interact with humans. <br=1> \n
    I am capable of showing different expressions. Like this, <br=1> \n
    <expression=happiness> <br=4> \n
    If someone makes me angry, <expression=anger:1.0> <br=4> \n
    when I see a gross thing, <expression=disgust:1.0> <br=4> \n
    <expression=happiness:0.5>when I'm scared, <expression=fear:1.0> <br=4> \n
    sometimes I feed sad, <expression=sadness> <br=4> \n
    when I suprised, <expression=surprise> <br=4> \n
    <expression=happiness:0.5> <br=3> \n
    How was my expressions? looks good? <br=1>
    '''
    
    return close(intent['sessionAttributes'],
                'Fulfilled',
                {'contentType': 'PlainText',
                'content': say})

def weather_handler(intent):
    weather_data = get_weather(city)
    
    current_city = weather_data['name']
    current_weather = weather_data['weather'][0]['main']
    current_weather_desc = weather_data['weather'][0]['description']
    current_temp = weather_data['main']['temp'] - 273.15 # Kelvin to Celcius
    current_wind_speed = weather_data['wind']['speed']

    output_string = random.choice(WEATHER_TEXT)
    say = "<expression=happiness:1.0>"
    say += output_string.format(city=current_city, current_weather_desc=current_weather_desc, current_temp=current_temp, current_wind_speed=current_wind_speed)
    say += " <br=1>"
    
    return close(intent['sessionAttributes'],
                'Fulfilled',
                {'contentType': 'PlainText',
                'content': say})

def goodbye_handler(intent):
    say = '''<expression=happiness:1.0>Thanks for comming. The team here, at the Univesity of Auckland are working on a project to develop a friendly receptionist robot. <br=1> \n
            I hope you enjoy your visit at the Univeristy of Auckland! Have a nice day! <br=1>
    '''
    return close(intent['sessionAttributes'],
                'Fulfilled',
                {'contentType': 'PlainText',
                'content': say})



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

    if intent_name == 'Welcome':
        return welcome_handler(intent_request)
    # elif intent_name == 'IntroStaffInfo':
    #     return intro_staff_handler(intent_request)
    elif intent_name == 'DemoFaceExpression':
        return demo_face_expression(intent_request)
    elif intent_name == 'WeatherToday':
        return weather_handler(intent_request)
    elif intent_name == 'EverThankYou':
        return goodbye_handler(intent_request)
    elif intent_name == 'IntroLab':
        return intro_lab_handler(intent_request)


    raise Exception('Intent with name ' + intent_name + ' not supported')

def lambda_handler(event, context):
    os.environ['TZ'] = 'America/Los_Angeles'
    time.tzset()
    logger.info('Received event: {}'.format(event))

    return dispatch(event)