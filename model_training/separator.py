import os
import shutil

if __name__ == '__main__':

    # OG_DATASET
    train_PATH = "WAID-images\\train"
    valid_PATH = "WAID-images\\valid"
    test_PATH = "WAID-images\\test"

    #ZEBRA_DATASET
    train_dest = "zebras_WAID_images\\train"
    valid_dest = "zebras_WAID_images\\valid"
    test_dest = "zebras_WAID_images\\test"

    #ZEBRA_LABELS
    train_label_dest = "zebras_WAID_labels\\train"
    valid_label_dest = "zebras_WAID_labels\\valid"
    test_label_dest = "zebras_WAID_labels\\test"

    labels = os.listdir(valid_label_dest)
    labels = [label[:-4] for label in labels]

    # for label in labels:
    #     print(f"{valid_PATH}\\{label}.jpg")
    #     shutil.copyfile(f"{valid_PATH}\\{label}.jpg", f"{valid_dest}\\{label}.jpg")