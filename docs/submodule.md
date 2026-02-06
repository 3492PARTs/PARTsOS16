# Submodules
This is meant for our developers, but anyone can use this information.

With the 2026 year is the new PARTs Library.\
The point being generic reusable code from season to season.
That's where PARTsLib comes in.\
It's pulled down as a submodule and build with the project as a sub-project and included as a dependency.\
This means that the PARTsLib code is built along with the project anytime you build or deploy.\
The only real downside to this implementation is the added complexity.

## Making Changes
In order to update the code in the library, you first have to make a new branch.\
![](img/submodules-1.png)

When changes are made, you commit and push to your new branch.

Watch out here! You now have to watch over which library branch you're using on your current project branch.\
This can quickly get confusing if you lose track of your branches.

It's recommended to push the branch to master/main as soon as you're done and ready to avoid confusion.\
As a general rule of thumb, you should be on the master/main branch of the library with the exception that you are actively working on or testing it.
```git submodule update --init -r```