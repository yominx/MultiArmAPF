




import json
 
# Opening JSON file
f = open('panda_tasks/000000.json')
data = json.load(f)
 
print(data) 
# Closing file
f.close()