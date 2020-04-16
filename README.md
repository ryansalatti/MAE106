The code of the simulation package given in MAE106 has been modified as following:

- Will run through 20 simulations in each of the three modes of control systems (60 simulations total)
- Each simulation is run on each of your CPU's cores (threading sims).
- Automatically logs the robot score into an array and exports the result into a CSV
- Removed GUI interface showing the robot running through the simulation

I have a 6 core PC and was able to get through all 60 simulations in around 5 minutes. I know some people spent multiple hours running and recording their results so hopefully this helps a lot of people save time.
