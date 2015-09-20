Package to get the baxter face animations working. The data folder has different faces and expressions used by Baxter. The src folder has two scripts:

1) animator_server_with_different_expressions.py -> listens to topics /emotion and /confusion/value/command, and publishes faces over /robot/xdisplay. Emotions set up right now are nonchalant, heard an order, up and down or the nod.

2) hear_words.py - run this after running the speech recognizer code and rosbridge. It hears data published on the /speech_recognition topic and publishes and emotion of heard an order over /emotion. Hence combining hear_words and animator_server_with_different_expressions might give different expressions we need.
