#include <stdio.h>  
#include <stdlib.h>
#include <math.h>

/* Declare All Functions */
int *findMax(int matrix[][3], int row, int col);
int *findMaxAbs(int matrix[][3], int row, int col);
int *findMaxOpt(int matrix[][3], int row, int col);

/* Driver Fucntion */
int main(int argc, char *argv[]) {  
    int matrix[3][3] = {{1, 2, 3}, {7, 8, 9}, {4, 5, 6}};  
  
    printf("Matrix:\n");  
    for (int i = 0; i < 3; i++) {  
        for (int j = 0; j < 3; j++) {  
            printf("%d ", matrix[i][j]);  
        }  
        printf("\n");  
    }  
    // General Function //
    int *final = findMax(matrix,3,3);
    int maxValue = final[0];
    int maxRow = final[1];
    int maxCol = final[2];
    printf("Max Value (General Method): %d @ [%d,%d]\n",maxValue,maxRow,maxCol);

    // Absolute Value Function //
    final = findMaxAbs(matrix,3,3);
    maxValue = final[0];
    maxRow = final[1];
    maxCol = final[2];
    printf("Max Value (Absolute Method): %d @ [%d,%d]\n",maxValue,maxRow,maxCol);

    // Optimal Finding Function //
    final = findMaxOpt(matrix,3,3);
    maxValue = final[0];
    maxRow = final[1];
    maxCol = final[2];
    printf("Max Value (Optimal Method): %d @ [%d,%d]\n",maxValue,maxRow,maxCol);

    return 0;  
}  

/* Functions */
int *findMax(int matrix[][3], int row, int col){
    int maxValue = 0;
    int maxRow;
    int maxCol;
    int temp;
    for (int i = 0; i < row; i++){
        for (int j = 0; j < col; j++){
            temp = matrix[i][j];
            if (temp > maxValue){
                maxValue = temp;
                maxRow = i + 1;
                maxCol = j + 1;
            } 
        }
    }
    int final[3] = {maxValue,maxRow,maxCol};
    return final;
}

int *findMaxAbs(int matrix[][3], int row, int col){
    int maxValue = 0;
    int maxRow;
    int maxCol;
    int temp;
    for (int i = 0; i < row; i++){
        for (int j = 0; j < col; j++){
            temp = abs(matrix[i][j]);
            if (temp > maxValue){
                maxValue = temp;
                maxRow = i + 1;
                maxCol = j + 1;
            } 
        }
    }
    int final[3] = {maxValue,maxRow,maxCol};
    return final;    
}

int *findMaxOpt(int matrix[][3], int row, int col){
    int midRow = 1;
    int midCol = 1;
    int j = 1;

    int tempMatrix[9] = {matrix[midRow][midCol],0,0,0,0,0,0,0,0};

    for (int i = midCol-1; i < 3; i++){
        tempMatrix[j] = matrix[midRow-1][i];
        j++;
    }

    for (int i = midCol-1; i < 3; i++){
        tempMatrix[j] = matrix[midRow+1][i];
        j++;
    }
 
    tempMatrix[j] = matrix[midRow][midCol-1];
    j++;

    tempMatrix[j] = matrix[midRow][midCol+1];
    j++;

    int tempTemp;
    int tempMax = 0;
    for (int i = 0; i < row*col; i++){
        tempTemp = tempMatrix[i];
        if (tempTemp > tempMax){
            tempMax = tempTemp;
        }
    }

    int maxValue = 0;
    int maxRow;
    int maxCol;
    int temp;
    for (int i = 0; i < row; i++){
        for (int j = 0; j < col; j++){
            temp = matrix[i][j];
            if (temp == tempMax){
                tempMax = temp;
                maxRow = i + 1;
                maxCol = j + 1;
            } 
        }
    }
    int final[3] = {tempMax,maxRow,maxCol};
    return final; 
}
