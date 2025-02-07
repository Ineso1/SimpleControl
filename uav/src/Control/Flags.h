#ifndef FLAGS_H
#define FLAGS_H

// #define CSV_STATE
// #define SAVE_ERRORS_CSV
// #define SAVE_STATE_ESTIMATION_CSV
// #define SAVE_REAL_STATE_SPACE_CSV

#define UDE_OBSERVER 1
#define LUENBERGER_OBSERVER 2
#define SUPERTWIST_OBSERVER 3
#define SLIDINGMODE_OBSERVER 4
#define ERRRORS_FILE_PATH_CSV     "/home/nessy/Documents/SimDataCSV/Errors.csv"   
#define TRANSLATION_FILE_PATH_CSV   "/home/nessy/Documents/SimDataCSV/RealStateSpace_trans.csv"
#define ROTATION_FILE_PATH_CSV      "/home/nessy/Documents/SimDataCSV/RealStateSpace_rot.csv"
#define DISTURBANCE_TRANSLATIONAL_FILE_PATH "/home/nessy/Documents/SimDataCSV/TranslationalEstimation.csv"
#define DISTURBANCE_ROTATIONAL_FILE_PATH    "/home/nessy/Documents/SimDataCSV/RotationalEstimation.csv"



// #define OBSERVER_TYPE SLIDINGMODE_OBSERVER
// #define OBSERVER_TYPE SUPERTWIST_OBSERVER
#define OBSERVER_TYPE UDE_OBSERVER
// #define OBSERVER_TYPE LUENBERGER_OBSERVER


#endif //FLAGS_H