# Python 3 code to rename multiple 
# files in a directory or folder
 
# importing os module
import os
 
# Function to rename multiple files
def main():
   
    folder = "checkerboard/8607_imgs"
    for count, filename in enumerate(os.listdir(folder)):
        dst = f"8607_{str(count)}.jpg"
        src =f"{folder}/{filename}"  # foldername/filename, if .py file is outside folder
        dst =f"checkerboard/{dst}"
         
        # rename() function will
        # rename all the files
        os.rename(src, dst)
 
# Driver Code
if __name__ == '__main__':
     
    # Calling main() function
    main()