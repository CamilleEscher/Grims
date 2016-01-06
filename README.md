<h1>Grims Project</h1>
<p><strong>Author : </strong>Camille ESCHER</p>

<p><strong>Start date : </strong>   10/17/15</br></p>

<p><strong>Main goal : </strong>Build a model and implement a prototype for turning the pages of a score when the musician is playing the end of the viewed staves (lines). </br>
<p><strong>Progress : </strong>The first step of the work focused on the correction of the slope of the score, then the position of the lines were extracted and the lines of the staves were removed. The bounding boxes process is in progress for now.</br></p>
<p><strong>To build the project : </strong></br>
Use 'make' to build </br>Use 'make doc' to generate the documentation</p>
<p><strong>To test it: </strong>Specify the executable './grims' and the path of a score as a first argument, then choose one ore more of the following arguments to see the results fo the processes :
<ul>
<li>printLines</li>
<li>resize</li>
<li>eraseLines</li>
<li>gatherStaves</li>
<li>printVerticalLines</li>
</ul>

<strong>References : </strong>
<ul>
<li><u>Reconnaissance de partitions musicales par modélisation floue des informations extraites et des règles de notation</u>, by Florence ROSSANT</b>
</li>
</ul>

<strong>Tools and language : </strong>
<ul>
<li>C++ (Language)</li>
<li>Opencv 3.0 (Library)</li>
<li>Makefile (Automation of compilation) (CMake later)</li>
<li>clang++ (Compiler version 3.6.0-2ubuntu1)</li>
<li>VIM (Editor)</li>
</ul>
