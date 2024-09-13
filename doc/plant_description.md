# Plant documentation
<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.7/MathJax.js?config=TeX-AMS-MML_HTMLorMML">
</script>
<script type="text/x-mathjax-config">
  MathJax.Hub.Config({
    TeX: { equationNumbers: { autoNumber: "AMS" } }
  });
</script>

<style>
    /* initialise the counter */
    body { counter-reset: figureCounter; }
    /* increment the counter for every instance of a figure even if it doesn't have a caption */
    figure { counter-increment: figureCounter; }
    /* prepend the counter to the figcaption content */
    figure figcaption:before {
        content: "Fig " counter(figureCounter) ": "
    }
</style>

Suppose we have an inverted pendulum with a cart described in the image below. 

<figure>
    <img src="img/inverted-pendulum.png" alt="Inverted pendulum system">
    <figcaption>Simple inverted pendulum with cart system</figcaption>
</figure>

The control force $u$ is applied to the cart. Assume that the center of gravity of the pendulum rod is at
its geometric center. We will obtain a mathematical model for the system. To analyze the dynamics of the system, we can draw a free body diagram like image below.

<figure>
    <img src="img/free-body-diagram.png" alt="Free body diagram of inverted pendulum">
    <figcaption>Free body diagram of inverted pendulum system</figcaption>
</figure>

Suppose that $(x_{G},y_{G})$ is the coordinates of
the center of gravity of the pendulum rod. 

Here is an equation:

$$
\begin{equation}
E = mc^2
\end{equation}
$$

$$
\begin{equation}
F = ma
\end{equation}
$$
