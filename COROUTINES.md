# Generators and Planning

You may be familiar with the classical definition of a program function, where
a function *return* implies the end of the function.

The only way couroutines differ from these regular functions is that they do not
typically end when they return something. In fact, what they do isn't referred
to as returning at all, but *yielding*.

Coroutines in Python are implemented using generators, and are for this reason
thought of as a stream of data; results from the function are available to the
caller as the function executes, rather than delivered in one chunk.

## Some Analogies

A common thing for a function to do is to call another function, do something
to the return, and then return that value. For example

```python
def now_utc():
    t = now()
    return as_utc(t)
```

For a coroutine, the equivalent thing is a little more complex,

```python
def ticks_utc():
    for t in ticks():
        yield as_utc(c)
```

## Generators as Vessels of Program State

Now, then, let us compare a simple moving routine for a robot. First the
classical style

```python
def move_to(target, speed=1e-1, tolerance=1e-2):
    while distance(get_position(), target) > tolerance:
        diff = get_position() - target
        set_velocity(diff/norm(diff)*speed)
```

This looks clean enough, and works great when no pre-emption is needed. The
only way to deal with pre-emption here is either by threading, or simply making
`move_to` only set the speed once.

Posit instead the following coroutine

```python
def move_to(target):
    while distance(get_position(), target) > tolerance:
        diff = get_position() - target
        set_velocity(diff/norm(diff)*speed)
        yield
```

With this simple addition, the function is now paused after each velocity is
calculated. This means the caller is free to interleave execution of this
`move_to` with other things, such as checking if moving to the current target
still makes sense. For example, path planning and replanning becomes fairly
straight-forward:

```python
def move_to_planned(target):
    path = plan_path(target)
    for subtarget in path:
        for v in move_to(subtarget):
            if will_collide():
                set_velocity(0)
                break
            yield
```

This can further be combined with logic for retrying, or changing plan once a
more optimal path is found.

## Dealing with Errors

Something that eventually comes up is a generator that needs to clean up its
state in the event of errors. Our `move_to`, for example, only sets velocities.
What would happen if something went wrong in our program? As it stands, it
would seem like the robot would just continue on its trajectory -- dangerous!

Let us modify the coroutine thus

```python
def move_to(target):
    try:
        while distance(get_position(), target) > tolerance:
            diff = get_position() - target
            set_velocity(diff/norm(diff)*speed)
            yield
    finally:
        set_velocity(0)
```

But wait, you think, we'll never reach the last line if something went wrong in
the layer above that is to unwind this coroutine. And you'd be right. What we
need is a context manager, like the following

```python
def move_to_planned(target):
    path = plan_path(target)
    for subtarget in path:
        with closing(move_to(subtarget)) as it:
            for v in it:
                if will_collide():
                    raise Collision()
                yield
```

The `with` statement here will make sure to close the coroutine, triggering
`GeneratorExit` at the yield statement and thus setting velocity to zero.

## Futher Reading

What I have described is a way to implement [behavior
trees](https://en.wikipedia.org/wiki/Behavior_tree) in pure Python. The ticks
of the tree are simply the unwinding of the generator.
