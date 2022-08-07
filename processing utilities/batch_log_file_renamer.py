import os, string, re
#7/8/2022 at 11:40:37 am, Steady mdoe

remove_duplicates = True
alpha_array = list(string.ascii_uppercase)
months = {1: "January", 2:"February",3:"March",4:"April",5:"May",6:"June",
          7:"July",8:"August",9:"September",10:"October",11:"November",12:"December"}

def decode_name(input_name):
    name_correct_format = re.search(r"^[A-Z]{1}[0-9]{2}[A-Z]{1}[0-9]{3}[BS]{1}.CSV$", input_name)
    if not name_correct_format:
        print(input_name)
        print("incorrect format for renaming\n")
        return False
    
    year = "2022"
    #print(input_name)
    month = months[alpha_array.index(input_name[0])]
    #month = "".join(("0", str(month))) if month < 10 else str(month)    #makes a string number for month

    day = input_name[1:3]
    
    hour = alpha_array.index(input_name[3])
    hour = "".join(("0", str(hour))) if hour < 10 else str(hour)
    
    minute = input_name[4:6]
    seconds = "".join((input_name[6], "0"))
    
    mode = "Burst" if input_name[7] == "B" else "Steady"
    
    #print(f"month:\t\t{month}")
    #print(f"day:\t\t{day}")
    #print(f"hour:\t\t{hour}")
    #print(f"minute:\t\t{minute}")
    #print(f"seconds:\t{seconds}")
    #print(f"mode:\t\t{mode}")
    
    new_name = (f"{month} {day}, {year} at {hour}-{minute}-{seconds} - {mode}.csv")
    #print(new_name)
    return new_name

#C:\Users\Evan\Desktop\renaming test
directory = input("Directory: ")
#directory = r"E:\test"
#print(directory)
os.chdir(directory)
print("\nAll directory contents: ")
print(os.listdir())

files_list = []
for item in os.listdir():
    if os.path.isfile(os.path.join(directory, item)):
        files_list.append(item)
print("\nFiles only")
print(files_list)

for file in files_list:
    decoded = decode_name(file)
    if decoded:       #if the file name wasn't decoded, returns false
        if not os.path.isfile(decoded):
            os.rename(file, decode_name(file))
        else:
            if remove_duplicates:
                os.remove(file)
        #os.rename(file, decode_name(file))
        
print("\nRenamed contents:\n")
for i in os.listdir():
    print(i)
