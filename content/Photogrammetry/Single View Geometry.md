$$
\underbrace{\textbf{x}}_{4\times1}=\underbrace{P}_{4\times4}\underbrace{\textbf{X}}_{4\times1}
$$
$$
\textbf{x}=K[R|t]\textbf{X}
$$
**In Expanded form:**
$$ 
\begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}
=
\begin{bmatrix} 
P_{11} & P_{12} & P_{13} & P_{14} \\
P_{21} & P_{22} & P_{23} & P_{24} \\
P_{31} & P_{32} & P_{33} & P_{34} \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} X \\ Y \\ Z \\ 1\end{bmatrix}
$$
$$ 
\begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}
=
\begin{bmatrix} 
K_{11} & K_{12} & K_{13} & 0 \\
K_{21} & K_{22} & K_{23} & 0 \\
K_{31} & K_{32} & K_{33} & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} 
R_{11} & R_{12} & R_{13} & t_{x} \\
R_{21} & R_{22} & R_{23} & t_{y} \\
R_{31} & R_{32} & R_{33} & t_{z} \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}
$$
Now,
- In $\begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}$, $\begin{bmatrix} x \\ y \\ z \end{bmatrix}$ represents a **scaled projection**. 
- Where, $z$ is actual depth of this point. 
- So, to the actual point on the camera image is:  $\begin{bmatrix} x/z \\ y/z \end{bmatrix}$, which can also be termed as **Image Coordinates**

> [!NOTE] Main Intution
> This term $z$ is lost when projection occours.  Which is the biggest problem in the field of Photogrammetry !

# Change in Coordinate system
![[Pasted image 20231008100526.png]]
# Intrinsic Parameters
- **Intrinsic matrix** is a transformation matrix that converts points from the camera coordinate system to the pixel coordinate system **(c2p)**
- The matrix $K$ is the intrinsic parameter:
$$
\begin{bmatrix} 
K_{11} & K_{12} & K_{13} \\
K_{21} & K_{22} & K_{23} \\
K_{31} & K_{32} & K_{33}
\end{bmatrix}
$$
- Scaling the parameters:
  ![[CamScanner 09-24-2023 19.58_1~2.jpg]]
# Change of Coordinate System
> [!NOTE]
> NOT SURE ABOUT THIS
> For two coordinate system A and B with $F$ representing the trasnsformation of coordinate system A->B,
> If $X_A$ and $X_B$ represents a point w.r.t  coordinate system A and B respectively,
> Then, $X_B=F^{-1}X_A$ 
>  i.e $X_A=FX_B$

> [!SUCCESS] To remember
> $X_A=$ [A->B] $X_B$
> 
# Extrinsic Parameters
- The **extrinsic matrix** is a transformation matrix from the world coordinate system to the camera coordinate system **(w2c)**
## Rotation
ToDo
## Translation
ToDo