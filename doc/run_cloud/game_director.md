Game Director Interface Instructions:

# Create Game Director User

A Game Director user must first be created to control a game within the Game Director Interface, to create a Game Director user run:

	> create_user.py

If a user is already created in the SQLite database (gdi.db), the following message will appear:

	> A user already exists! Create another? (y/n): 

Enter 'y', without the tick marks and then enter an email address for the user when prompted.

	> Enter email address: 

Press enter/return and then enter a Password when prompted.

	> Password: 

Press enter/return and then retype the password entered above when prompted.

	> Password (again): 

Press enter/return and the password match and the user does not exist, the new user will be created.


@ Log in as Game Director User

To login as a Game Director, launch the application server:

	For SSL/TLS:
	> python run.py --ssl --cert <server.crt> --key <server.key>

	For non SSL/TLS:
	> python run.py

Once the application is running, navigate to the Game Director Interface

http(s)://hostname:hostport

	hostname = hostname running the Game Director Application Server [defaults 127.0.01]
	hostport = host port that the Game Director Application Server is listening on [defaults 5001]

When the web pages loads, click on the Menu button (represented by three (3) horizontal lines stacked on top of each other) in the top left hand corner of the page.

Enter the username of the Game Director user created above.
** NOTE **: username will be the first portion of the email address before the '@' symbol.  For example, if email address entered was test@testmail.com, usernaem will be 'test' withouth the tick marks.

Enter the password of the user entered.

Once username and password have been typed in, then click the Login button.  Login dialog will disappear if login was successful.



# Create a Game

When a valid Game Director user has logged in successfully, a 'Create Game' button will appear.

Click the 'Create Button' to create a new game.

Enter values for 'Game Type', 'Team 1', and 'Team 2' in the Create Game dialog by selecting the drop-down button and selecting the appopriate choice:

Choices are:
Game Type: <LIVE>, <VIRTUAL>
Team 1: <USAFA>, <USMA>, <USNA>
Team 2: <USAFA>, <USMA>, <USNA>
Duration: a value in minutes (defaults to 30 minutes)

** NOTE **:  Team 1 equates to 'blue' team and Team 2 equates to 'red' in the ACS software suite.

When values have been selected, click on the 'Create New Game' button.

If no errors occur during creating a game, the 'Start Game' and 'End Game' buttons should appear.


# Starting / Stopping a Game

Once a game has been successfully created, the 'Start Game' and 'End Game' buttons appear.

To start a game, simply press the 'Start Game' button.

When the game has been started, the countdown clock will start counting down from the entered duration above, the game progress indicator will start, the score for each team will be displayed.

** NOTE **: At this time, not knowing what Arbiter will do when it receives the start message, an offset value is stored for each team to zero out the existing score of the Arbiter.

To end a game, simply press the 'End Game' button.

When the game has ended, score will be saved in the SQLite database and a 'Reset Game' button will appear.


# Reset a Game

When a game has ended, the Game Director can reset all values / scores by pressing the 'Reset Game' button.

When the 'Reset Game' button is pressed, the Game Director can create a new game (as mentioned above) and start that game.
