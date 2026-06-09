# Introduction to Inverse Kinematics

Considering only revolute joints.

Represent the $n$ joint angles as $\theta = (\theta_{1},\theta_{2},\dots, \theta_{n})^{\mathrm{T}}$, and the positions of the $k$ end-effectors as $s = (s_{1},s_{2},\dots,s_{k})^{\mathrm{T}}$. The position of each end-effector is a function of the joint angles, i.e., $s_{i} = s_{i}(\theta)$.

The target positions are represented as $t = (t_{1},t_{2},\dots,t_{k})^{\mathrm{T}}$.

Inverse Kinematics (IK) aims to find a set of joint angles $\theta$ such that $s_{i}(\theta) = t_{i}$. Since robot rotations are typically described using trigonometric functions (sines and cosines), the process of finding solutions is essentially solving a system of non-linear equations.

To solve this non-linear system, a first-order derivative can be used for linear approximation:

$$ f(\theta + \Delta\theta) = f(\theta) + \frac{ \partial f }{ \partial \theta }|_{\theta} \Delta\theta + O(\Delta\theta^2)$$

$$ \dot{s} = J(\theta)\dot{\theta}$$

This leads to the introduction of the **Jacobian matrix** $J$.

In discrete time, the equation becomes:

$$ \Delta s \approx J \Delta \theta$$

Therefore, the goal within this discrete time step is to find a $\Delta \theta$ such that $J\Delta \theta = t-s$.

## Jacobian Transpose Method

To avoid computing the matrix inverse, the iterative update is defined as:

$$ \Delta \theta = \alpha J^{\mathrm{T}}e$$

#### Proof of Validity

When $\Delta \theta = \alpha J^{\mathrm{T}} e$, the change in position is $\Delta s \approx \alpha J J^{\mathrm{T}} e$.

Taking the inner product of the displacement vector and the error vector $e$ for any Jacobian matrix $J$:

$$ \langle JJ^{\mathrm{T}}e,e\rangle = \langle J^{\mathrm{T}}e,J^{\mathrm{T}}e \rangle = \lVert J^{\mathrm{T}}e \rVert^2 \geq 0$$

This implies that for any $J$ and $e$, the angle between the resulting displacement vector and the error vector is less than 90 degrees, which guarantees a continual reduction in the magnitude of the error vector.

The optimal step size $\alpha$ is generally chosen as:

$$ \alpha = \frac{\langle e,JJ^{\mathrm{T}}e \rangle}{\langle J J^{\mathrm{T}}e, J J^{\mathrm{T}}e \rangle}$$

## Pseudoinverse Method

The pseudoinverse method seeks a direct solution to $e = J\Delta \theta$ by utilizing the Moore-Penrose pseudoinverse of $J$ (denoted as $J^{\dagger}$) to obtain $\Delta \theta$:

$$ \Delta \theta = J^{\dagger}e$$

The pseudoinverse method possesses two primary optimization properties:

1. **In reachable cases**, it yields the joint increment $\Delta \theta$ with the minimum norm ($\lVert \Delta \theta \rVert$).
    
2. **In unreachable cases**, it provides the solution that minimizes the residual error $\lVert J\Delta\theta-e \rVert$.
    

Additionally, the null space can be exploited via the pseudoinverse. For any arbitrary vector $\phi$, the relationship $J(I-J^{\dagger}J)\phi = 0$ holds true. The projection matrix $I-J^{\dagger}J$ projects any joint velocity vector into the null space of $J$, meaning it produces zero movement at the end-effector.

Consequently, the general solution can be formulated as:

$$ \Delta \theta = J^{\dagger}e + (I-J^{\dagger}J)\phi$$

## Damped Least Squares (DLS) Method

Compared to the pseudoinverse method, Damped Least Squares (DLS) no longer solely pursues the minimization of $\lVert J\Delta\theta - e \rVert ^2$. Instead, it introduces a trade-off to consider the stability of the current configuration by minimizing the following objective function:

$$ E(\Delta \theta) = \lVert J\Delta\theta-e \rVert ^2 + \lambda^2 \lVert \Delta\theta \rVert^2$$

#### Mathematical Derivation

$$
\begin{align} 
E(\Delta \theta) &= \frac{1}{2}(J\Delta\theta-e)^{\mathrm{T}}(J\Delta\theta-e) + \frac{1}{2} \lambda^2\Delta\theta ^{\mathrm{T}}\Delta\theta \\ 
&=\frac{1}{2}(\Delta\theta ^{\mathrm{T}}J^{\mathrm{T}}J\Delta\theta-\Delta\theta ^{\mathrm{T}}J^{\mathrm{T}}e-e^{\mathrm{T}}J\Delta\theta+e^{\mathrm{T}}e)+\frac{1}{2}\lambda^2\Delta\theta ^{\mathrm{T}}\Delta\theta \\ \\ 
\frac{ \partial E }{ \partial \Delta\theta } & = J^{\mathrm{T}}J\Delta\theta-J^{\mathrm{T}}e+\lambda^2\Delta\theta = 0 \\ \\ 
\therefore & (J^{\mathrm{T}}J+\lambda^2I)\Delta\theta = J^{\mathrm{T}}e \\ 
& \Delta\theta = (J^{\mathrm{T}}J + \lambda^2I)^{-1}J^{\mathrm{T}}e \end{align}
$$

However, in practice, directly inverting the $n \times n$ matrix $(J^{\mathrm{T}}J + \lambda^2I)$ can be computationally expensive (especially for highly redundant manipulators where $n \gg k$).

We can transform the matrix using the following identity (proven by left-multiplying both sides by $(J^{\mathrm{T}}J + \lambda^2I)$ and right-multiplying by $(JJ^{\mathrm{T}} + \lambda^2I)$ to yield identical expanded forms):

$$ (J^{\mathrm{T}}J + \lambda^2I)^{-1}J^{\mathrm{T}} = J^{\mathrm{T}}(JJ^{\mathrm{T}} + \lambda^2I)^{-1}$$

Thus, the more computationally efficient form is:

$$ \Delta\theta = J^{\mathrm{T}}(JJ^{\mathrm{T}}+\lambda^2I)^{-1}e$$

#### Alternative Derivation via Augmented Matrix

Another elegant way to formulate this is to recast the objective function as the $L_2$ norm of an augmented matrix:

$$ \left\| \begin{pmatrix} J \\ \lambda I \end{pmatrix}\Delta\theta - \begin{pmatrix} e \\ 0 \end{pmatrix} \right\|^2$$

Applying the standard normal equation for linear least squares, $(A^{\mathrm{T}}A)x = A^{\mathrm{T}}b$, where $A = \begin{pmatrix} J \\ \lambda I \end{pmatrix}$ and $b = \begin{pmatrix} e \\ 0 \end{pmatrix}$:

$$ \begin{pmatrix} J \\ \lambda I \end{pmatrix}^{\mathrm{T}} \begin{pmatrix} J \\ \lambda I \end{pmatrix} \Delta \theta = \begin{pmatrix} J \\ \lambda I \end{pmatrix}^{\mathrm{T}} \begin{pmatrix} e \\ 0 \end{pmatrix}$$

Noting that $\begin{pmatrix} J \\ \lambda I \end{pmatrix}^{\mathrm{T}} = \begin{pmatrix} J^{\mathrm{T}} & \lambda I \end{pmatrix}$, the block multiplication yields:

$$ (J^{\mathrm{T}}J + \lambda^2I)\Delta\theta = J^{\mathrm{T}}e$$

$$ \Delta\theta = (J^{\mathrm{T}}J + \lambda^2I)^{-1}J^{\mathrm{T}}e$$

Various dynamic adjustment algorithms exist for the strategic selection of the damping factor $\lambda$.

## Comparison via Singular Value Decomposition (SVD)

The fundamental difference between the pseudoinverse and the DLS method can be clearly analyzed through the lens of Singular Value Decomposition (SVD).

Let the SVD of the Jacobian matrix be:

$$ J = U D V^{\mathrm{T}} = \sum^{m}_{i=1}\sigma_{i}u_{i}v_{i}^{\mathrm{T}}$$

Where $\sigma_i$ represents the singular values. Thus, the pseudoinverse $J^{\dagger}$ is expressed as:

$$ J^{\dagger} = V D ^{\dagger}U^{\mathrm{T}} = \sum^{m}_{i=1}\sigma_{i}^{-1}v_{i}u_{i}^{\mathrm{T}}$$

Similarly, performing SVD analysis on the DLS formulation $J^{\mathrm{T}}(JJ^{\mathrm{T}}+\lambda^2I)^{-1}$:

$$ JJ^{\mathrm{T}} + \lambda^2I = (UDV^{\mathrm{T}})(VD^{\mathrm{T}}U^{\mathrm{T}}) + \lambda^2I = U(DD^{\mathrm{T}}+\lambda^2I)U^{\mathrm{T}}$$

Since $DD^{\mathrm{T}}+\lambda^2I$ is a diagonal matrix with diagonal entries equal to $\sigma_{i}^2 + \lambda^2$, its inverse is also a diagonal matrix with entries $\frac{1}{\sigma^2_{i}+\lambda^2}$.

Substituting this back gives:

$$ J^{\mathrm{T}}(JJ^{\mathrm{T}}+\lambda^2I)^{-1} = VD^{\mathrm{T}}(DD^{\mathrm{T}}+\lambda^2I)^{-1}U^{\mathrm{T}}=\sum_{i=1}^m \frac{\sigma_{i}}{\sigma_{i}^2+\lambda^2}v_{i}u_{i}^{\mathrm{T}}$$

#### Key Insights

- **Pseudoinverse Coefficient:** $\frac{1}{\sigma_{i}}$
    
- **Damped Least Squares Coefficient:** $\frac{\sigma_{i}}{\sigma_{i}^2+\lambda^2}$
    

This comparison reveals that when the manipulator is far from singularities (i.e., the singular values $\sigma_i$ are relatively large), both methods perform similarly. However, as the robot approaches a kinematic singularity ($\sigma_i \to 0$), the coefficient of the pseudoinverse method spikes toward infinity, causing joint velocity explosion. Conversely, the DLS coefficient smoothly goes to $0$, bounded by the damping factor, effectively preventing excessively large joint steps ($\Delta \theta$).

In physical terms, the singular values $\sigma_i$ in the SVD of the Jacobian represent the "transmission ratio" or gain along a specific directional axis. A larger $\sigma_i$ means that small joint movements easily generate substantial displacement at the end-effector in that direction. At a singularity point, $\sigma_i$ drops to $0$, meaning the mechanical structure completely loses the ability to move along that particular degree of freedom.
