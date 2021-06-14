# Shell script to spawn a new docker container
# with the files of the repository
docker build -t automatic_fuzzing .
docker run -it automatic_fuzzing 
