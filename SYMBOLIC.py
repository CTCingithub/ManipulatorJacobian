import sympy as p


def Translation_4x4(Displacement):
    # Calculate Transformation Matrix in translation cases
    return (
        p.Identity(3)
        .as_explicit()
        .row_join(Displacement)
        .col_join(p.Matrix([[0, 0, 0, 1]]))
    )


def Rotation_RPY_4x4(RPY):
    # psi, theta, phi = RPY[0], RPY[1], RPY[2]
    return (
        MatrixExp_4x4(p.Matrix([1, 0, 0, 0, 0, 0]), RPY[0])
        @ MatrixExp_4x4(p.Matrix([0, 1, 0, 0, 0, 0]), RPY[1])
        @ MatrixExp_4x4(p.Matrix([0, 0, 1, 0, 0, 0]), RPY[2])
    )


def TransformationMatrix_Inverse(TransformationMatrix):
    R_matrix = TransformationMatrix[:3, :3]
    p_vector = TransformationMatrix[3, :3].reshape(3, 1)
    return R_matrix.T.row_join(-R_matrix.T @ p_vector).col_join(
        p.Matrix([[0, 0, 0, 1]])
    )


def Vector2Matrix_3x3(Vector):
    # Convert 3x1-shaped \hat{\omega} vectors to 3x3-shaped
    # [\hat{\omega}] matrixes
    #! Sympy doesn't support reshapes like .reshape(-1,?)
    vec = Vector.reshape(3, 1)
    return p.Matrix(
        [
            [0, -vec[2, 0], vec[1, 0]],
            [vec[2, 0], 0, -vec[0, 0]],
            [-vec[1, 0], vec[0, 0], 0],
        ]
    )


def MatrixExp_3x3(Vector, Angle):
    # Calculate exp([\hat{\omega}] \theta) matrixes, \hat{\omega} is a
    # 3x1-shaped vector
    mat_temp = Vector2Matrix_3x3(Vector)
    return (
        p.Identity(3)
        + mat_temp * p.sin(Angle)
        + mat_temp @ mat_temp * (1 - p.cos(Angle))
    ).as_explicit()


def Joint2Twist(Joint, Location):
    # Calculate twists from joint information
    return Joint.col_join(Vector2Matrix_3x3(Location) @ Joint)


def PVector(Twist, Angle):
    # Calculate \vec{p}s
    omega = p.Matrix(Twist[:3, :]).reshape(3, 1)
    v = p.Matrix(Twist[3:, :]).reshape(3, 1)
    return (p.Identity(3).as_explicit() - MatrixExp_3x3(omega, Angle)) @ (
        Vector2Matrix_3x3(omega) @ v
    ) + omega @ omega.T @ v * Angle


def MatrixExp_4x4(Twist, Angle):
    # Calculate exp([\hat{\xi}] \theta) matrixes, \hat{\xi} is a
    # 6x1-shaped twist
    UpperLeft = MatrixExp_3x3(Twist[:3, :], Angle)
    UpperRight = PVector(Twist, Angle)
    return UpperLeft.row_join(UpperRight).col_join(p.Matrix([[0, 0, 0, 1]]))


def Twist2Matrix_4x4(Twist):
    return (
        Vector2Matrix_3x3(Twist[:3, :])
        .row_join(Twist[3:, :])
        .col_join(p.Matrix([[0, 0, 0, 0]]))
    )


def AdjointMatrix(TransMatrix):
    R = TransMatrix[:3, :3]
    P = TransMatrix[:-1, -1]
    return p.simplify(
        R.row_join(p.zeros(3, 3)).col_join((Vector2Matrix_3x3(P) @ R).row_join(R))
    )


def AdjointInverseMatrix(TransMatrix):
    R = TransMatrix[:3, :3]
    P = TransMatrix[:-1, -1]
    return p.simplify(
        R.T.row_join(p.zeros(3, 3)).col_join(
            (-R.T @ Vector2Matrix_3x3(P)).row_join(R.T)
        )
    )
