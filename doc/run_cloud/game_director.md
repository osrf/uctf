Game Director Interface Instructions:

# Log in as Game Director User


Enter the username of the Game Director user created above.

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
