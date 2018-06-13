#!/usr/bin/python
import os
import json

import rospy
import rospkg
import yaml

import uuid
import boto3

from mind_msgs.msg import Reply, RaisingEvents
from mind_msgs.srv import ReloadWithResult, ReadData, WriteData

class LexHandler: 
    def __init__(self):
        rospy.init_node('lex_intent_handler', anonymous=False)
        try:
            config_file = os.path.join(
                rospkg.RosPack().get_path('lex_dialog'), 'config', 'config.yaml')
        except:
            rospy.logerr('please set param config.yaml')
            exit(-1)
        # open config for lex
        with open(config_file) as f:
            data = yaml.load(f)
        self.client = boto3.client(
            'lex-runtime',
            region_name=data['region'],
            aws_access_key_id=data['aws_access_key_id'],
            aws_secret_access_key=data['aws_secret_access_key']
        )
        self.speech = ''
        self.session_id = uuid.uuid4().hex
        self.session_attributes = {}

        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raise_events)
        self.pub_reply = rospy.Publisher('reply', Reply, queue_size=10)

        rospy.loginfo('[%s] Initialzed'%rospy.get_name())
        rospy.spin()

    def handle_raise_events(self, msg):
        if msg.recognized_word != '':
            response = self.client.post_content(
                botName = 'LabCurator',
                botAlias = 'lab_curator',
                userId = self.session_id,
                contentType = 'text/plain; charset=utf-8',
                sessionAttributes = self.session_attributes,
                accept = 'text/plain; charset=utf-8',
                inputStream = msg.recognized_word)

            reply_msg = Reply()
            reply_msg.header.stamp = rospy.Time.now()
            reply_msg.reply = response.get('message')
            
            self.pub_reply.publish(reply_msg)
        else:
            rospy.loginfo('getting signal from guide bot')

if __name__ == '__main__':
    l = LexHandler()

