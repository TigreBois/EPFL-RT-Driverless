## Using git:
First: [read this blog](https://rogerdudler.github.io/git-guide/), it’s a good introduction and it quickly covers the basic.
For the repository:
  1) Go to the team’s repository
  2) Click on Code and copy the https link
  3) On your machine, navigate in a terminal to an empty folder where you want to put the repository’s content. Then, enter this command: `git clone linkYouCopied` This will install the repository’s content on your machine, and only needs to be done once, ideally.

### Managing branches:

We will work using branches, they are described in the blog linked at the beginning. Ideally, it's good to have a branch per feature/task. The current policy is that to merge a branch back to master, all conflicts must have been resovled, and two people have to review the changes.

To create a branch:
1) To create a new branch, type `git checkout -b nameOfBranch` in your local repository. For now, the branch you created is only local; you have to push and commit it to the repository for others to see it.
2) Type `git commit -am "message or name of the branch"` to commit the newly created branch.
3) Finally, type `git push -u origin nameOfBranch` to push it to the online repository, so that it can be visible by everyone. Now you can work on this branch however you like; Then, use `git add files` / `git add * ` to stage your changes, `git commit` to commit them locally, and `git push` to push them online.
4) It's good to also create a new pull request at this point and link it to your branch, so that the tools on github can work on it.

To merge a branch back to master:
1) If it wasn't done before, create a pull request on the online repository, associated to your branch.
2) Make sure you commit and push all the changes you made.
3) Pull the changes that have happened to master while you were working on your branch with `git pull origin master`; this will likely causes conflicts if the same bits of code you're working on have been modified and merged by someone else; you will need to fix these conflicts. Git usually writes the part coming from master and the part coming from your code between lines of # where there has been a conflict.
4) Once all conflicts have been resovled, assigne two people to review your changes on the pull request's page.
5) If everything is in order, and they validate your pull request, you can merge it to master. Then, you can either delete the branch, or continue working on it; you will have to create another pull request to merge new changes to master regardless.
6) Some other requierments to merge might be necessary depending on the tools we add to the repository; if you have any question or problem, don't hesitate to ask.

Here's how the typical workflow happens using git: Once you've setup your branch, you work locally as you would without git. 
Once you've made some progress, you open a terminal at the root of the repository, and add the changes you've made; you can add them one by one, or use `git add -A` to add everything.
Then, you enter `git commit` to commit them locally; it will open a file in a text editor, where you write the commit message, a.k.a. a quick summary of the changes you've made. (I encourage you to look up how to write good commit messages, they are important to keep track of what was done)
Finally, you can enter `git push` to push them to the online repository. It is good to also enter `git pull` before pushing to make sure you have the latest version of your branch.