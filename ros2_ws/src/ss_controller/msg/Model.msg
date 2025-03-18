# discrete time step
float64 dt
#discrete Model in kalman canonical form
# state matrix
Matrix Ad

# input matrix
Matrix Bd

# output matrix
Matrix Cd

# similaraliry matrix
Matrix T

# inverse similarity matrix
Matrix T_inv

# info for controllability and observability
int32 n_co
int32 n__co
int32 n_c_o
int32 n__c_o