
file_entry = "input.txt"
#file_exit = "output.msg"

<<<<<<< HEAD
Dictionnary = {"fp32":"float32", "fp64":"float64", "u64":"int64", "u32":"int32", "u16":"int16", "u8":"int8"}
=======
Dictionnary = {"fp32":"float32", "fp64":"float64", "u64":"int64", "u32":"int32", "u16":"int16", "u8":"int8", "s8":"int8"}
>>>>>>> develop


def IsNotId(variable): #this function check if the line is right
    try:
        int(variable)
        return False
    except ValueError:
        return True



def MsgTranslator(file_entry):
    with open(file_entry, 'r') as entry:
        lines = entry.readlines()
        words_first_line = lines[0].split()
    print(createName(words_first_line))
    file_exit = "Subsonus" + createName(words_first_line) + ".msg"


    with open(file_exit, 'w') as exitData:
        for line in lines:
            if checkNotInterestingLine(line): #if the line is not a usuable one but only text or space, let's quit
                #exitData.write("WARNING THERE IS A PROBLEM" + "\n") #Uncomment it to
                continue

            words = line.split()
            name_variable = createName(words)
            type_variable = Dictionnary[words[2]]
            exitData.write(type_variable + " " + name_variable + "\n")

    print("Treatment has finished. Information have been written in ", file_exit)

def createName(words):
    name_variable = ""
    cpt = 0
    for k in range(4, len(words)-1):  # Put len(words)-1 if you don't want the units
        word = words[k]
        if "(" in word:
            word = word[1:]
        if ")" in word:
            word = word[:-1]

        if cpt > 0:
            name_variable += "_"
        name_variable += word.lower()
        cpt+=1
    return name_variable

def checkNotInterestingLine(line):
    words = line.split()
    if (len(words)) <= 4 or IsNotId(words[0]):  # check if the line is different from usual
        return True

MsgTranslator(file_entry)