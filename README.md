# EPFL-RT Driverless Section

Github repository for the Driverless Section

## Using git :
First : [read this blog](https://rogerdudler.github.io/git-guide/), it’s a good introduction and it quickly covers the basic.
For the repository:
  1) Go to the team’s repository
  2) Click on Code and copy the https link
  3) On your machine, navigate in a terminal to an empty folder where you want to put
     the repository’s content. Then, enter this command : |git clone linkYouCopied|
This will install the repository’s content on your machine, and only needs to be done
once, ideally.

### Managing branches :

We will work using branches, they are described in the blog linked at the beginning. Ideally,
it's good to have a branch per feature/task. The current policy is that to merge a branch back
to master, all conflicts must have been resovled, and two people have to review the
changes.

To create a branch:
1) To create a new branch, type |git checkout -b nameOfBranch| in your local repository ;
for now, the branch you created is only local.
2) Type |git commit -am "message or name of the branch"| to commit the newly created
branch.
3) Finally, type |git push -u origin nameOfBranch| to push it to the online repository, so
that it can be visible by everyone.
Now you can work on this branch however you like; Then, use |git add files| / |git add * | to
stage your changes, |git commit| to commit them locally, and |git push| to push them online.

To merge a branch back to master:
1) Create a pull request on the online repository, associated to your branch.
2) Commit and push all the changes you made to your branch.
3) Pull the changes that have happened to master while you were working on your
branch with |git pull origin master| ; this will likely causes conflicts if the same bits of
code you're working on have been modified and merged by someone else; you will
need to fix these conflicts. Git usually writes the part coming from master and the
part coming from your code between lines of # where there has been a conflict.
4) Once all conflicts have been resovled, assigne two people to review your changes on
the pull request's page.
5) If everything is in order, and they validate your pull request, you can merge it to
master. Then, you can either delete the branch, or continue working on it; you will
have to create another pull request to merge new changes to master regardless.
