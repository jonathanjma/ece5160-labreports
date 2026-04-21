// get the ninja-keys element
const ninja = document.querySelector('ninja-keys');

// add the home and posts menu items
ninja.data = [{
    id: "nav-about",
    title: "about",
    section: "Navigation",
    handler: () => {
      window.location.href = "/ece5160-labreports/";
    },
  },{id: "nav-labs",
          title: "labs",
          description: "All my lab reports for the semester!",
          section: "Navigation",
          handler: () => {
            window.location.href = "/ece5160-labreports/labs/";
          },
        },{id: "labs-lab-1",
          title: 'Lab 1',
          description: "Artemis and Bluetooth",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab1.html";
            },},{id: "labs-lab-10",
          title: 'Lab 10',
          description: "Grid Localization using Bayes Filter (Simulation)",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab10.html";
            },},{id: "labs-lab-2",
          title: 'Lab 2',
          description: "Inertial Measurement Unit (IMU)",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab2.html";
            },},{id: "labs-lab-3",
          title: 'Lab 3',
          description: "Time of Flight (ToF)",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab3.html";
            },},{id: "labs-lab-4",
          title: 'Lab 4',
          description: "Motors and Open Loop Control",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab4.html";
            },},{id: "labs-lab-5",
          title: 'Lab 5',
          description: "Linear PID Control and Extrapolation",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab5.html";
            },},{id: "labs-lab-6",
          title: 'Lab 6',
          description: "Orientation Control",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab6.html";
            },},{id: "labs-lab-7",
          title: 'Lab 7',
          description: "Kalman Filter",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab7.html";
            },},{id: "labs-lab-8",
          title: 'Lab 8',
          description: "Stunts!",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab8.html";
            },},{id: "labs-lab-9",
          title: 'Lab 9',
          description: "Mapping",
          section: "Labs",handler: () => {
              window.location.href = "/ece5160-labreports/labs/lab9.html";
            },},{
        id: 'social-email',
        title: 'email',
        section: 'Socials',
        handler: () => {
          window.open("mailto:%6A%6A%6D%34%39%38@%63%6F%72%6E%65%6C%6C.%65%64%75", "_blank");
        },
      },{
        id: 'social-github',
        title: 'GitHub',
        section: 'Socials',
        handler: () => {
          window.open("https://github.com/jonathanjma", "_blank");
        },
      },{
      id: 'light-theme',
      title: 'Change theme to light',
      description: 'Change the theme of the site to Light',
      section: 'Theme',
      handler: () => {
        setThemeSetting("light");
      },
    },
    {
      id: 'dark-theme',
      title: 'Change theme to dark',
      description: 'Change the theme of the site to Dark',
      section: 'Theme',
      handler: () => {
        setThemeSetting("dark");
      },
    },
    {
      id: 'system-theme',
      title: 'Use system default theme',
      description: 'Change the theme of the site to System Default',
      section: 'Theme',
      handler: () => {
        setThemeSetting("system");
      },
    },];
