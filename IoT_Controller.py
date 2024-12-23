import paho.mqtt.client as mqtt
import json
import time

class IoT_Controller:
    client = None
    #JSON : JavaScript Object Notation is the format used for the rules below
    # [] lists of items go between []
    # {} lists of key-value pairs go between {} (dictionaries)
    rules = []
    mqtt_data = {}
    # to remember the messages we sent out
    message_log = []

    def configure():
        filename = "rules.json"
        with open(filename,'r') as file:
            IoT_Controller.rules = json.load(file)

        IoT_Controller.client = mqtt.Client()
        #pass the reference to the callback function to handle incoming messages
        IoT_Controller.client.on_message = IoT_Controller.on_message
        #must connect to the MQTT message broker at "localhost" on port 1883
        IoT_Controller.client.connect("localhost",1883)
#****************************************************************************
        # The topic wildcards to use when subscribing are
        # "+" for a single level, e.g., house/+ will catch house/temp but not /house/temp/room
        # "#" for a multiple levels, e.g., # will catch house/temp,  /house/temp/room, and bad/bad/cat
        IoT_Controller.client.subscribe("#")
#****************************************************************************

    def on_message(client, userdata, message):
        #this is where we handle the messages
        #Using try..except (exception handling)
        try:
            value = float(message.payload.decode("utf-8"))
        except ValueError:
            print("String")
            value = message.payload.decode("utf-8")
        topic = message.topic

        #discriminate messages that I sent vs messages that I did not send
        #if IoT_Controller.message_log contains an entry
        for entry in IoT_Controller.message_log:
            if entry["time"] < time.time() - 5:
                #delete old message_log entries
                IoT_Controller.message_log.remove(entry)
            elif entry["topic"] == topic and entry["value"] == value:
                return

        #record the received data in our dictionary, replacing any older value for the same topic
        IoT_Controller.mqtt_data[topic] = value

        #the only action is the printout
        print(topic, value)

        #based on the value(s) on the topic(s) received trigger an action

        #loop through the rules
#		{
#			"condition":{"topic":"house/temp", "comparison":">", "value":30},
#			"action":{"message":"It's too hot, turn on the AC", "topic":"room/AC", "value":"on"}
#		},

        for rule in IoT_Controller.rules:
            conditions = rule["conditions"] #changed from condition
            conditions_met = True
            for condition in conditions:
                #use the topic from the condition to access the value in the mqtt_data dictionary
                topic = condition["topic"]
                try:
                    value = IoT_Controller.mqtt_data[topic] #not going to work if there is no value for the key provided
                    condition_met = IoT_Controller.condition_met(value,condition["comparison"],condition["value"])
                except KeyError:
                    value = None
                    condition_met = False
                conditions_met = conditions_met and condition_met

            if conditions_met:
                #action
                action = rule["action"]
                print(action["message"])
                IoT_Controller.client.publish(action["topic"],action["value"])
                #record that we sent that message
                entry = {"time":time.time(), "topic":action["topic"], "value":action["value"]}
                IoT_Controller.message_log.append(entry) #end the item to the end of the list


    def condition_met(value,comp_operator,comp_value):
        if comp_operator == ">":
            return value > comp_value
        if comp_operator == ">=":
            return value >= comp_value
        if comp_operator == "<":
            return value < comp_value
        if comp_operator == "<=":
            return value <= comp_value
        if comp_operator == "==":
            return value == comp_value

    def run():
        IoT_Controller.client.loop_forever()

def main():
    IoT_Controller.configure()
    IoT_Controller.run()

if __name__ == "__main__":
    main()
