# Forward Diffusion Process
$\beta_t$ : Amount of noise added at the step $t$ w.r.t step $t-1$. 
$\beta_t\in(0,1)$  

$\alpha_t = 1-\beta_t$ : Amount of image information preserved at the step $t$ w.r.t step $t-1$. $\alpha_t \in (1,0)$

$\bar \alpha_t = \prod_{i=1}^{t} \alpha_i$ : Amount of image information preserved at the step $t$ w.r.t step 0. $\bar \alpha_t \in (1,0)$

Now,
$$x_t = \sqrt {\bar \alpha_t} x_0 + \sqrt{1-\bar \alpha_t} \epsilon $$
$$q(x_t|x_0) = \mathcal{N}(x_t, \sqrt {\bar \alpha_t} x_0, (1-\bar \alpha_t) I)$$
where, $\epsilon$ = Random values in the range of 0-1


> [!WARNING] Initial value of $\beta$
> Should the first term of $\beta$ be 0 ? because if not , first value of $\alpha$ will be < 1 , which means , even $x_0$ will have noise in it, But I think $x_0$ is the initial input image !!!

# Reverse Diffusion Process
TODO
